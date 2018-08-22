/**
 * @file Compress.cpp
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief dynamic range compression
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

#include <math.h>
#include "Compress.h"

namespace jl {

//============================= PEAK DETECTORS ===============================//

//--------------------------- BASE PEAK DETECTOR -----------------------------//

void
PeakDetector::setSamplingRate(float sr) {
  samplingRate = sr;
  setAttack(msAttack);
  setRelease(msRelease);
}

void
PeakDetector::setAttack(float a) {
  msAttack = a;
  alphaAttack = alphaFromTau(msAttack * 0.001);
  if (extendRelease) updateAlphaRelease();
}

void
PeakDetector::setRelease(float r) {
  msRelease = r;
  updateAlphaRelease(); // always
}

////////// PRIVATE

float
PeakDetector::alphaFromTau(float tau) {
  return exp(-1 / (JL_MAX(tau * samplingRate, 1)));
}

void
PeakDetector::updateAlphaRelease() {
  float realRelease = extendRelease ? (msAttack + msRelease) : msRelease;
  alphaRelease = alphaFromTau(realRelease * 0.001);
}

//--------------------------- BRANCHING SMOOTHED -----------------------------//

float
BranchingSmoothedPeakDetector::process(float in) {
  if (in > prevYL) {
    YL = alphaAttack * prevYL + (1 - alphaAttack) * in;
  } else {
    YL = alphaRelease * prevYL + (1 - alphaRelease) * in;
  }

  return YL;
}

//--------------------------- DECOUPLED SMOOTHED -----------------------------//

float
DecoupledSmoothedPeakDetector::process(float in) {
  prevY1 = Y1;
  prevYL = YL;

  Y1 = fmax(in, alphaRelease * prevY1 + (1 - alphaRelease) * in);
  YL = alphaAttack * prevYL + (1 - alphaAttack) * Y1;

  return YL;
}

//================================ SIDE CHAIN ================================//

//--------------------------- LOG DOMAIN DETECTOR ----------------------------//

// maybe too many calls to clipInfinity due to paranoia,
// but they're not VERY expensive
// TODO : check which ones to remove

sample
LogDomainSideChain::process(sample in) {
  // XG = atodb<double>(avg.process(fabs(static_cast<double>(in))));
  XG = atodb<double>(fabs(static_cast<double>(in)));
  
  YG = gc.process(XG);
  XL = static_cast<float>(clipInfinity<float>(XG - YG));
  YL = clipInfinity<float>(pd.process(XL));
  Cdb = clipInfinity<float>(makeUp - YL);

  return static_cast<sample>(dbtoa<float>(Cdb));
}

sample
LogDomainSideChain::process(sample in, sample m, sample t, sample r, sample k) {
  makeUp = static_cast<float>(m);
  gc.setThreshold(static_cast<float>(t));
  gc.setRatio(static_cast<float>(r));
  gc.setKnee(static_cast<float>(k));

  return process(in);
}

//=========================== DB ENVELOPE FOLLOWER ===========================//

// maybe too many calls to clipInfinity due to paranoia,
// but they're not VERY expensive
// TODO : check which ones to remove

sample
LogDomainFlattener::process(sample in1, sample in2) {
  double in1Amp = static_cast<double>(avg1.process(fabs(in1)));
  double in2Amp = static_cast<double>(avg2.process(fabs(in2)));

  // empirical silence value : 1e-18
  // (Average doesn't always return to zero due to residuals)
  if (in1Amp <= 1e-18 || in2Amp <= 1e-18) return 0;

  // thresh is defined by the first input's dB value :
  thresh = atodb<double>(in1Amp);
  XG = atodb<double>(in2Amp);  

  gc.setThreshold(static_cast<float>(clipInfinity<float>(thresh)));
  YG = gc.process(XG);

  XL = static_cast<float>(clipInfinity<float>(XG - YG));
  YL = clipInfinity<float>(pd.process(XL));
  Cdb = clipInfinity<float>(makeUp - YL);

  return static_cast<sample>(dbtoa<float>(Cdb));
}

sample
LogDomainFlattener::process(sample in1, sample in2, sample m, sample r, sample k) {
  makeUp = static_cast<float>(m);
  gc.setRatio(static_cast<float>(r));
  gc.setKnee(static_cast<float>(k));

  return process(in1, in2);  
}

//================================ COMPRESSOR ================================//

void
Compressor::setSamplingRate(float sr) {
  samplingRate = sr;
  updateSamplingRate();
}

void
Compressor::setMakeUp(float m) {
  rMakeUp.ramp(m, rampSamples);
}

void
Compressor::setThreshold(float t) {
  rThreshold.ramp(t, rampSamples);
}

void
Compressor::setRatio(float r) {
  rRatio.ramp(r, rampSamples);
}

void
Compressor::setKnee(float k) {
  rKnee.ramp(k, rampSamples);
}

void
Compressor::setAttack(float a) {
  sc.setAttack(a);
}

void
Compressor::setRelease(float r) {
  sc.setRelease(r);
}

void
Compressor::process(sample **ins, sample **outs, unsigned int blockSize) {
  // TODO : check that blockSize < JL_MAX_BLOCK_SIZE

  sample *m = rMakeUp.process(blockSize);
  sample *t = rThreshold.process(blockSize);
  sample *r = rRatio.process(blockSize);
  sample *k = rKnee.process(blockSize);

  switch (compressorDesign) {
    case FeedforwardCompressorDesign:
    default:
      for (unsigned int i = 0; i < blockSize; ++i) {
        double gain = sc.process(
          static_cast<double>(ins[channels][i]),
          static_cast<float>(*(m + i)),
          static_cast<float>(*(t + i)),
          static_cast<float>(*(r + i)),
          static_cast<float>(*(k + i))
        );

        for (unsigned int c = 0; i < channels; ++c) {
          outs[c][i] = ins[c][i] * gain;
        }
      }
      break;
  }
}

////////// PRIVATE

void
Compressor::updateSamplingRate() {
  sc.setSamplingRate(samplingRate);
  rampSamples = static_cast<unsigned long>(rampDuration * samplingRate * 0.001);
}

//================================ DB FOLLOWER ===============================//

// TODO ...

} /* end namespace jl */