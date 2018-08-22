/**
 * @file OnePole.h
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief one pole lowpass filer
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

#ifndef _JL_DISTORT_H_
#define _JL_DISTORT_H_

#include <math.h>

namespace jl {

// simple pow function based overdrive (waveshaper)

template <typename T>
class Overdrive {
private:
  float factor;

public:
  Overdrive(float f = 1) :
  factor(f) {}

  ~Overdrive() {}

  void setFactor(float f) {
    factor = JL_MAX(f, 1e-9f);
  }

  T process(T in) {
    if (in >= 1) { return 1; }
    if (in <= -1) { return -1; }

    if (in >= 0) { return 1f - pow(1f - in, factor); }
    if (in < 0) { return pow(1f + in, factor) - 1f; }
  }
};

// decimator class similar to max's [degrade~] object
// TODO

template <typename T>
class Decimate {
private:
  // put a buffer here to compute downsampling internally (if really needed)
  // maybe use some accumulators / counters / similar tricks
  // try drop sample, linear (and more evoluted) interpolation

public:
  Decimate() {}
  ~Decimate() {}

  void setSamplingRate(float sr) {}
  void setResamplingFactor(float f) {}
  void setRequantizationFactor(float f) {}
  T process(T in) {}
};

} /* end namespace jl */

#endif /* _JL_DISTORT_H_ */