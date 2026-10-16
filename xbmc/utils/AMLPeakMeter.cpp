/*
 *  Copyright (C) 2026 Team CoreELEC
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "AMLPeakMeter.h"

#include "ServiceBroker.h"
#include "cores/DataCacheCore.h"
#include "utils/ScreenshotAML.h"
#include "platform/linux/SysfsPath.h"
#include "utils/log.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace
{
constexpr const char* PEAK_PARAM = "/sys/module/amdolby_vision/parameters/xbmc_dv_new_hdr10_meas_peak";
constexpr const char* CAPTURE_DEVICE = "/dev/amvideocap0";
constexpr unsigned long AMVIDEOCAP_IOW_SET_WANTFRAME_WIDTH = _IOW('V', 0x02, int);
constexpr unsigned long AMVIDEOCAP_IOW_SET_WANTFRAME_HEIGHT = _IOW('V', 0x03, int);

constexpr auto PERIOD = std::chrono::milliseconds(125); // ~8 captures per second
constexpr int SAMPLES = 32000;                           // maxRGB samples per capture
constexpr float PERCENTILE = 0.9995f;                    // ignore a few isolated pixels
constexpr int64_t HOLD_MS = 2000;                        // rise at once, hold the window max
constexpr float DECAY = 0.3f;                            // then close 30 % of the gap per sample
const float STEP = std::log(1.05f);                      // write on a 5 % change only

// PQ (ST 2084) code, full range 8 bit as the capture delivers it -> nits
std::array<float, 256> BuildNits()
{
  std::array<float, 256> t{};
  for (int c = 0; c < 256; ++c)
  {
    const double e = std::pow(c / 255.0, 1.0 / 78.84375);
    const double v = std::max(e - 0.8359375, 0.0) / (18.8515625 - 18.6875 * e);
    t[c] = static_cast<float>(10000.0 * std::pow(v, 1.0 / 0.1593017578125));
  }
  return t;
}
const std::array<float, 256> NITS = BuildNits();

int64_t NowMs()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
} // namespace

CAMLPeakMeter::CAMLPeakMeter() : CThread("AMLPeakMeter")
{
}

CAMLPeakMeter::~CAMLPeakMeter()
{
  Stop();
}

void CAMLPeakMeter::Start(int srcWidth, int srcHeight, float masteringPeak)
{
  Stop();
  // Requests smaller than the double-write copy are cropped, not scaled.
  m_width = (srcWidth / 4) & ~31;
  m_height = (srcHeight / 4) & ~1;
  if (m_width <= 0 || m_height <= 0 || !CSysfsPath(PEAK_PARAM).Exists())
    return;
  m_step = std::max(1, static_cast<int>(std::sqrt(m_width * m_height / static_cast<float>(SAMPLES))));
  m_cap = masteringPeak > 0.0f ? masteringPeak : 10000.0f;
  m_written = -1;
  m_out = 0.0f;
  m_window.clear();
  m_flush = false;
  CLog::Log(LOGINFO, "CAMLPeakMeter: measuring the HDR10 peak on {}x{} captures, step {}",
            m_width, m_height, m_step);
  Create();
}

void CAMLPeakMeter::Stop()
{
  if (IsRunning())
    StopThread(true);
  if (m_written > 0)
    Write(0);
}

bool CAMLPeakMeter::Measure(float& nits)
{
  const int size = m_width * m_height * 3;
  std::vector<uint8_t> buf(size);
  ssize_t got = -1;
  {
    std::lock_guard<std::mutex> lock(CScreenshotAML::CaptureMutex());
    const int fd = open(CAPTURE_DEVICE, O_RDWR);
    if (fd < 0)
      return false;
    if (ioctl(fd, AMVIDEOCAP_IOW_SET_WANTFRAME_WIDTH, m_width) == 0 &&
        ioctl(fd, AMVIDEOCAP_IOW_SET_WANTFRAME_HEIGHT, m_height) == 0)
      got = pread(fd, buf.data(), size, 0); // EAGAIN at playback start/seek
    close(fd);
  }
  if (got != size)
    return false;

  std::array<int, 256> hist{};
  int count = 0;
  for (int y = 0; y < m_height; y += m_step)
  {
    const uint8_t* p = buf.data() + y * m_width * 3;
    for (int x = 0; x < m_width; x += m_step, p += 3 * m_step)
    {
      hist[std::max({p[0], p[1], p[2]})]++;
      count++;
    }
  }
  // A compressed-only frame read as NV21 is noise, mostly saturated; the first
  // capture after open also is, with fewer saturated pixels but a 10000-nit
  // percentile that the 2 s hold would then keep. No real grade sits there.
  if (!count || hist[255] > count / 4 || hist[255] > static_cast<int>(count * (1.0f - PERCENTILE)))
    return false;

  int left = static_cast<int>(count * (1.0f - PERCENTILE));
  for (int c = 255; c >= 0; --c)
  {
    left -= hist[c];
    if (left < 0)
    {
      nits = NITS[c];
      return true;
    }
  }
  nits = 0.0f;
  return true;
}

void CAMLPeakMeter::Write(int nits)
{
  CSysfsPath peak{PEAK_PARAM};
  if (peak.Exists())
    peak.Set(nits);
  m_written = nits;
}

void CAMLPeakMeter::Process()
{
  while (!m_bStop)
  {
    const auto start = std::chrono::steady_clock::now();
    if (m_flush.exchange(false))
    {
      m_window.clear();
      m_out = 0.0f;
    }

    // HDR10+ converted to DV also starts as HDR10: the kernel only applies the
    // peak to HDR10 input, but keep the param at 0 for anything else.
    if (CServiceBroker::GetDataCacheCore().GetVideoHdrType() != StreamHdrType::HDR_TYPE_HDR10)
    {
      if (m_written != 0)
        Write(0);
      m_window.clear();
      m_out = 0.0f;
    }
    else
    {
      float p;
      if (Measure(p))
      {
        const int64_t now = NowMs();
        p = std::min(p, m_cap);
        // first sample of a playback/seek: take it as is, nothing to decay from
        if (m_window.empty())
          m_out = p;
        m_window.emplace_back(now, p);
        while (!m_window.empty() && m_window.front().first < now - HOLD_MS)
          m_window.pop_front();
        float wmax = 0.0f;
        for (const auto& s : m_window)
          wmax = std::max(wmax, s.second);
        m_out = wmax >= m_out ? wmax : m_out - (m_out - wmax) * DECAY;
        const int q = std::max(
            100, static_cast<int>(std::lround(std::exp(std::round(std::log(std::max(m_out, 1.0f)) / STEP) * STEP))));
        // re-read: a manual A/B may have reset it
        CSysfsPath peak{PEAK_PARAM};
        if (q != m_written || peak.Get<int>().value_or(-1) != q)
        {
          if (q != m_written)
            CLog::Log(LOGDEBUG, "CAMLPeakMeter: scene peak {:.0f} nits -> {}", p, q);
          Write(q);
        }
      }
    }

    const auto spent = std::chrono::steady_clock::now() - start;
    if (spent < PERIOD)
      Sleep(std::chrono::duration_cast<std::chrono::milliseconds>(PERIOD - spent));
  }
}
