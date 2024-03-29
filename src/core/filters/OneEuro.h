/**
 * @file OneEuro.h
 * @author Joseph Larralde
 * @date 29/07/2023
 * @brief one euro lowpass filter (see Casiez, Roussel & Vogel, 1 euro filter :
 * a simple speed-based low-pass filter for noisy input in interactive systems)
 *
 * @copyright
 * Copyright (C) 2023 by Joseph Larralde.
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


#ifndef _JL_ONEEURO_H_
#define _JL_ONEEURO_H_

#include <math.h>

namespace jl {

template <typename T>
class OneEuro {
private:
  float samplingRate;
  float cutoffFrequency;
  double a0, b1, z1;

public:
  OneEuro(float fc = 20) :
  samplingRate(44100), cutoffFrequency(fc), z1(0) {
    updateCoefficients();
  }

  ~OneEuro() {}

  void setSamplingRate(float sr) {
    samplingRate = sr;
    updateCoefficients();
  }

  void setCutoffFrequency(float f) {
    cutoffFrequency = f;
    updateCoefficients();
  }

  T process(T in) {
    z1 = static_cast<float>(in) * a0 + z1 * b1;
    return static_cast<T>(z1);
  }

private:
  updateCoefficients() {
    b1 = exp(-2.0 * M_PI * cutoffFrequency / samplingRate);
    a0 = 1.0 - b1;    
  }
};

} /* end namespace jl */

#endif /* _JL_ONEEURO_H_ */