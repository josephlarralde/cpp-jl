/**
 * @file Biquad.h
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief biquad filter class with lowpass, highpass, bandpass, notch and allpass modes
 *
 * mostly taken from :
 * https://www.musicdsp.org/en/latest/_downloads/Audio-EQ-Cookbook.txt
 *
 *          Cookbook formulae for audio EQ biquad filter coefficients
 * ----------------------------------------------------------------------------
 *            by Robert Bristow-Johnson  <rbj@audioimagination.com>
 * etc ...
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

#ifndef _JL_BIQUAD_H_
#define _JL_BIQUAD_H_

#include "../../jl.h"

namespace jl {

enum BiquadMode {
  LowpassBiquadMode = 0,
  HighpassBiquadMode,
  BandpassBiquadMode,
  NotchBiquadMode,
  AllpassBiquadMode
};

class Biquad {
private:
  value fs, f0, q;
  BiquadMode mode;
  sample a0, a1, a2, b0, b1, b2;
  sample w0, cosw0, sinw0, alpha;
  /*
    these variables are needed to compute the "direct form 1" :
    y[n] = (b0/a0)*x[n] + (b1/a0)*x[n-1] + (b2/a0)*x[n-2]
                        - (a1/a0)*y[n-1] - (a2/a0)*y[n-2]            (Eq 4)
  */
  sample b0norm, b1norm, b2norm, a1norm, a2norm;
  sample xMinusOne, xMinusTwo;
  sample yMinusOne, yMinusTwo;


public:
  Biquad() :
  fs(44100), f0(100), q(0),
  mode(LowpassBiquadMode),
  a0(0), a1(0), a2(0), b0(0), b1(0), b2(0),
  w0(0), cosw0(0), sinw0(0), alpha(0),
  b0norm(0), b1norm(0), b2norm(0), a1norm(0), a2norm(0),
  xMinusOne(0), xMinusTwo(0),
  yMinusOne(0), yMinusTwo(0) {}

  ~Biquad() {}

  void setSamplingRate(value f) {
    fs = f;
    updateCoefficients();
  }

  void setF(value f) {
    if (f != f0) {
      f0 = f;
      updateCoefficients();
    }
  }

  void setQ(value f) {
    if (f != q) {
      q = f;
      updateCoefficients();
    }
  }

  void setMode(BiquadMode m) {
    mode = m;
    updateCoefficients();
  }

  sample process(sample x) {
    sample y = b0norm * x +
               b1norm * xMinusOne +
               b2norm * xMinusTwo -
               a1norm * yMinusOne -
               a2norm * yMinusTwo;

    xMinusTwo = xMinusOne;
    xMinusOne = x;
    yMinusTwo = yMinusOne;
    yMinusOne = y;

    return y;
  }

  void reset() {
    xMinusOne = xMinusTwo = 0;
    yMinusOne = yMinusTwo = 0;
  }

private:
  void updateCoefficients() {
    w0 = 2 * M_PI  * f0 / fs;
    cosw0 = cos(w0);
    sinw0 = sin(w0);
    alpha = sinw0 / (2 * q);

    switch (mode) {
      case LowpassBiquadMode:
        b0 = (1 - cosw0) / 2;
        b1 = 1 - cosw0;
        b2 = b0;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
        break;
      case HighpassBiquadMode:
        b0 = (1 + cosw0) / 2;
        b1 = -(1 + cosw0);
        b2 = b0;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
        break;
      case BandpassBiquadMode:
        // constant skirt gain, peak gain = Q
        // b0 = sinw0 / 2; // = q * alpha;
        // b1 = 0;
        // b2 = -sinw0 / 2; // = -q * alpha;
        // a0 = 1 + alpha;
        // a1 = -2 * cosw0;
        // a2 = 1 - alpha;

        // constant 0 dB peak gain
        b0 = alpha;
        b1 = 0;
        b2 = -alpha;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
        break;
      case NotchBiquadMode:
        b0 = 1;
        b1 = -2 * cosw0;
        b2 = 1;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
        break;
      case AllpassBiquadMode:
        b0 = 1 - alpha;
        b1 = -2 * cosw0;
        b2 = 1 + alpha;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
        break;
      default: // no way
        break;
    }

    b0norm = b0 / a0;
    b1norm = b1 / a0;
    b2norm = b2 / a0;
    a1norm = a1 / a0;
    a2norm = a2 / a0;
  }
};

} /* end namespace jl */

#endif /* _JL_BIQUAD_H_ */
