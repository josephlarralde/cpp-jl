/**
 * @file IntervalMap.h
 * @author Joseph Larralde
 * @date 14/08/2018
 * @brief map an input interval to an output interval using transfer functions
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

#ifndef _JL_INTERVALMAP_H_
#define _JL_INTERVALMAP_H_

#include <math.h>
#include "../../jl.h"

namespace jl {

template <typename T>
class IntervalMap {
private:
  T inputMin;
  T inputMax;
  T outputMin;
  T outputMax;

  T xFactor, realXFactor;
  T sFactor, realSFactor;

  T epsilon;
  bool applyXFactor;
  bool applySFactor;

  bool constantOutput;
  T constantValue;

  T a, b;
  T axIn, bxIn;
  T axOut, bxOut;
  T asIn, bsIn;
  T asOut, bsOut;
  T asToxIn, bsToxIn;
  T axTosIn, bxTosIn;

public:
  IntervalMap() :
  inputMin(0), inputMax(1), outputMin(0), outputMax(1),
  xFactor(1), sFactor(1),
  epsilon (1e-9), applyXFactor(false), applySFactor(false),
  constantOutput(false), constantValue(0),
  a(1), b(0), axIn(1), bxIn(0), axOut(1), bxOut(0),
  asIn(2), bsIn(-1), asOut(0.5), bsOut(0.5),
  asToxIn(0.5), bsToxIn(0.5), axTosIn(2), bxTosIn(-1) {}

  ~IntervalMap() {}

  void setInputMin(T iMin) {
    inputMin = iMin;
    updateCoefficients();
  }

  void setInputMax(T iMax) {
    inputMax = iMax;
    updateCoefficients();
  }

  void setOutputMin(T oMin) {
    outputMin = oMin;
    updateCoefficients();
  }

  void setOutputMax(T oMax) {
    outputMax = oMax;
    updateCoefficients();
  }

  void setXFactor(T xf) {
    xFactor = xf;
    applyXFactor = fabs(xf) > epsilon;
    realXFactor = 1 + pow(fabs(xf), 2);
  }

  void setSFactor(T sf) {
    sFactor = sf;
    applySFactor = fabs(sf) > epsilon;
    realSFactor = 1 + pow(fabs(sf), 2);
  }

  T process(T in) {
    if (constantOutput) { return constantValue; }

    if (!applySFactor && !applyXFactor) {
      return a * in + b;
    }

    T clipped = (inputMax > inputMin)
              ? JL_CLIP(in, inputMin, inputMax)
              : JL_CLIP(in, inputMax, inputMin);

    T tmpIn, tmpOut;

    // if only xFactor applied

    if (!applySFactor) {
      tmpIn = axIn * clipped + bxIn;
      if (xFactor >= 0) {
        tmpOut = pow(fabs(tmpIn), realXFactor);
      } else {
        tmpOut = 1 - pow(1 - fabs(tmpIn), realXFactor);
      }

      return axOut * tmpOut + bxOut;    
    }

    // if only sFactor applied

    if (!applyXFactor) {
      tmpIn = asIn * clipped + bsIn;
      if (sFactor >= 0) {
        tmpOut = 1 - pow(1 - fabs(tmpIn), realSFactor);
      } else {
        tmpOut = pow(fabs(tmpIn), realSFactor);
      }

      if (tmpIn < 0) { tmpOut *= -1; }

      return asOut * tmpOut + bsOut;
    }

    // if both factors applied

    tmpIn = asIn * clipped + bsIn;
    if (sFactor >= 0) {
      tmpOut = 1 - pow(1 - fabs(tmpIn), realSFactor);
    } else {
      tmpOut = pow(fabs(tmpIn), realSFactor);
    }

    if (tmpIn < 0) { tmpOut *= -1; }

    tmpIn = asToxIn * tmpOut + bsToxIn;
    if (xFactor >= 0) {
      tmpOut = pow(fabs(tmpIn), realXFactor);
    } else {
      tmpOut = 1 - pow(1 - fabs(tmpIn), realXFactor);
    }

    return axOut * tmpOut + bxOut;

    // does this even make sense for mapping ?
    // for now, negative sFactor values are clipped to zero in jl/map pd external
    // if both factors applied (order changing according to sFactor sign):

    // if (sFactor >= 0) {
    //   tmpIn = asIn * clipped + bsIn;
    //   if (sFactor >= 0) {
    //     tmpOut = 1 - pow(1 - fabs(tmpIn), realSFactor);
    //   } else {
    //     tmpOut = pow(fabs(tmpIn), realSFactor);
    //   }

    //   if (tmpIn < 0) { tmpOut *= -1; }

    //   tmpIn = asToxIn * tmpOut + bsToxIn;
    //   if (xFactor >= 0) {
    //     tmpOut = pow(fabs(tmpIn), realXFactor);
    //   } else {
    //     tmpOut = 1 - pow(1 - fabs(tmpIn), realXFactor);
    //   }

    //   return axOut * tmpOut + bxOut;
    // } else {
    //   tmpIn = axIn * clipped + bxIn;
    //   if (xFactor >= 0) {
    //     tmpOut = pow(fabs(tmpIn), realXFactor);
    //   } else {
    //     tmpOut = 1 - pow(1 - fabs(tmpIn), realXFactor);
    //   }

    //   tmpIn = axTosIn * tmpOut + bxTosIn;
    //   tmpOut = pow(fabs(tmpIn), realSFactor);

    //   if (tmpIn < 0) { tmpOut *= -1; }

    //   return asOut * tmpOut + bsOut;
    // }
  }

private:
  void updateCoefficients() {
    if (fabs(inputMax - inputMin) < epsilon) {
      constantOutput = true;
      constantValue = inputMax;
    } else if (fabs(outputMax - outputMin) < epsilon) {
      constantOutput = true;
      constantValue = outputMax;
    } else {
      constantOutput = false;

      a = (outputMax - outputMin) / (inputMax - inputMin);
      b = outputMax - a * inputMax;

      axIn = 1 / (inputMax - inputMin);
      bxIn = 1 - axIn * inputMax;

      axOut = (outputMax - outputMin);
      bxOut = outputMax - axOut;

      asIn = 2 / (inputMax - inputMin);
      bsIn = 1 - asIn * inputMax;

      // these never change
      // asToxIn = 0.5;
      // bsToxIn = 0.5;
      // axTosIn = 2;
      // bxTosIn = -1;

      asOut = (outputMax - outputMin) * 0.5;
      bsOut = outputMax - asOut;
    }
  }
};

} /* end namespace jl */

#endif /* _JL_INTERVALMAP_H_ */
