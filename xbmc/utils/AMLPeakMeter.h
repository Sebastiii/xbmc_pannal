/*
 *  Copyright (C) 2026 Team CoreELEC
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "threads/Thread.h"

#include <atomic>
#include <deque>
#include <utility>

// Measures the scene peak of HDR10 video on the decoded picture and hands it
// to the kernel (xbmc_dv_new_hdr10_meas_peak): HDR10 carries no per-scene peak,
// so the newer Dolby library would otherwise tone-map for the mastering display.
// The capture (/dev/amvideocap0) only sees the source while the HEVC decoder
// also writes its uncompressed 1/4 copy (double_write_mode 3), which the codec
// sets up before it opens the decoder.
class CAMLPeakMeter : private CThread
{
public:
  CAMLPeakMeter();
  ~CAMLPeakMeter() override;

  // srcWidth/srcHeight: coded size; the capture is the double-write copy (1/4 per side).
  void Start(int srcWidth, int srcHeight);
  void Stop();
  // Seek/reset: forget the previous scene.
  void Flush() { m_flush = true; }

protected:
  void Process() override;

private:
  bool Measure(float& nits);
  void Write(int nits);

  int m_width = 0;
  int m_height = 0;
  int m_step = 1;
  int m_written = -1;
  float m_out = 0.0f;
  std::deque<std::pair<int64_t, float>> m_window;
  std::atomic<bool> m_flush{false};
};
