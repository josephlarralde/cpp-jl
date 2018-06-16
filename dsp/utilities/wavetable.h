/**
 * @file wavetable.h
 * @author Joseph Larralde
 * @date 07/06/2018
 * @brief set of interpolation functions and wavetable generators
 *
 * @copyright
 * Copyright (C) 2018 by Joseph Larralde.
 * All rights reserved.
 *
 * License (BSD 3-clause)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _JL_DSP_WAVETABLE_H_
#define _JL_DSP_WAVETABLE_H_

#include <cmath>

//============================ INTERPOLATORS =================================//

// p is minimum the 1sr sample of a buffer
// and maximum the (n - 1)th sample of the same buffer
// (or loop over buffer length ?)
static inline float interpolateLinear(float *buf, unsigned int index, float frac) {
  float *p = buf + index;

  return (1. - frac) * (*p) + frac * (*(p + 1));
}

static inline void interpolateLinearStride(float *buf, float *res, unsigned int index, float frac, unsigned int size) {
  float *p = buf + index;

  for (unsigned int i = 0; i < size; ++i) {
    *(res + i) = (1. - frac) * (*(p + i)) + frac * (*(p + i + size));
  }
}

// p is minimum the 2nd sample of a buffer
// and maximum the (n - 2)th sample of the same buffer
// (or loop over buffer length ?)
// use zero-padding wherever needed
static inline float interpolateBicubic(float *buf, unsigned int index, float frac) {
  float *p = buf + index;
  float a, b, c, d, cminusb;

  a = (index == 0) ? 0 : *(p - 1);
  b = *p;
  c = *(p + 1);
  d = *(p + 2);
  cminusb = c - b;

  return b + frac * (
                      cminusb - 0.1666667f * (1. - frac) *
                      ((d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b))
                    );
}

static inline void interpolateBicubicStride(float *buf, float *res, unsigned int index, float frac, unsigned int size) {
  float *p = buf + index;
  float a, b, c, d, cminusb;

  for (unsigned int i = 0; i < size; ++i) {
    a = (index == 0) ? 0 : *(p + i - size);
    b = *(p + i);
    c = *(p + i + size);
    d = *(p + i + 2 * size);
    cminusb = c - b;

    *(res + i) = b + frac * (
      cminusb - 0.1666667f * (1. - frac) *
      ((d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b))
    );
  }
}

//============================== GENERATORS ==================================//

static inline void generateSine(float *p, int nsamples) {
  for (int i = 0; i < nsamples; ++i) {
    *(p + i) = sin((float)i * 2.0f * M_PI / nsamples);
  }
}

static inline void generateHann(float *p, int nsamples) {
  for (int i = 0; i < nsamples; ++i) {
    *(p + i) = 0.5 * (1 - cos(((float)i * 2.0f * M_PI) / (nsamples - 1)));
  }
}

#endif /* _JL_DSP_WAVETABLE_UTILITIES_H_ */
