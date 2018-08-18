/**
 * @file Compress.h
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

#ifndef _JL_COMPRESS_H_
#define _JL_COMPRESS_H_

// mostly based on the article :
// "Digital Dynamic Range Compressor Design - A Tutorial and Analysis",
// by DIMITRIOS GIANNOULIS, MICHAEL MASSBERG, AND JOSHUA D. REISS,
// Queen Mary University of London

#include "../../jl.h"
#include "../utilities/units.h"
#include "../utilities/Ramp.h"
#include "../utilities/Average.h"

#include "m_pd.h"

namespace jl {

//============================= GAIN COMPUTER ================================//

// there is a single way of computing this,
// given by eq (4) in the reference article

template <typename T>
class GainComputer {
private:
  float threshold;
  float ratio;
  float oneOverRatio;
  float knee;

public:
  GainComputer() : threshold(0), ratio(1), oneOverRatio(1), knee(0) {}
  ~GainComputer() {}

  void setThreshold(float t) {
    threshold = t;
  }

  void setRatio(float r) {
    ratio = JL_MAX(r, 1);
    oneOverRatio = 1 / ratio;
  }

  void setKnee(float k) {
    knee = k;
  }

  T process(T in) {
    T limit = 2 * (in - threshold);
    T res;

    if (limit <= knee * -1) {
      res = in;
    } else if (limit > knee) {
      res = threshold + (in - threshold) * oneOverRatio;
    } else {
      T tmp = in - threshold + knee * 0.5;
      res = in + (oneOverRatio - 1) * tmp * tmp / (2 * knee);
    }

    return res;
  }

  T process(T in, T t, float r, float k = 0) {
    threshold = t;
    ratio = JL_MAX(r, 1);
    oneOverRatio = 1 / ratio;
    knee = k;

    return process(in);
  }
};

//============================ LEVEL DETECTORS ===============================//

// A peak detector is basically a sophisticated kind of IIR filter behaving
// differently when the input value increases and when it decreases.
// It models the analog circuitry used in hardware compressors to generate the
// attack and release ramps.
// As explained in the reference article, several approaches exist for the
// implementation a peak detector :
// - branching
// - decoupled
// - branching smoothed
// - decoupled smoothed

// The RMS level detector has to be a separate class because it has no
// attack / release parameters, only a single smooth time constant parameter,
// so it cannot implement the same interface.

//--------------------------- BASE PEAK DETECTOR -----------------------------//

class PeakDetector {
protected:
  bool extendRelease;

  float samplingRate;
  float msAttack;
  float msRelease;
  float alphaAttack;
  float alphaRelease;

  float YL;
  float prevYL;

public:
  PeakDetector(bool er = true) :
  extendRelease(er),
  samplingRate(44100),
  msAttack(1),
  msRelease(1),
  alphaAttack(1),
  alphaRelease(1),
  YL(0),
  prevYL(0) {}

  ~PeakDetector() {}

  virtual void setSamplingRate(float sr);
  virtual void setAttack(float a);
  virtual void setRelease(float r);

  virtual float process(float in) = 0;

protected:
  float alphaFromTau(float tau);
  void updateAlphaRelease();
};

//--------------------------- BRANCHING SMOOTHED -----------------------------//

class BranchingSmoothedPeakDetector : public PeakDetector {
public:
  BranchingSmoothedPeakDetector() : PeakDetector() {}
  virtual ~BranchingSmoothedPeakDetector() {}
  virtual float process(float in);
};

//--------------------------- DECOUPLED SMOOTHED -----------------------------//

class DecoupledSmoothedPeakDetector : public PeakDetector {
private:
  float Y1;
  float prevY1;

public:
  DecoupledSmoothedPeakDetector() : PeakDetector(), Y1(0), prevY1(0) {}
  virtual ~DecoupledSmoothedPeakDetector() {}
  virtual float process(float in);
};

//=============================== SIDE-CHAIN =================================//

// The topology of the side chain (level detector placement, which component
// works in which domain, etc) is independent from the global compressor design.
// We could use any side chain topology in all the following classical designs :
// - feedforward
// - feedback
// - alternate feedback (see article, allows to side chain an external signal)

// The various topologies described in the article are :
// - return-to-zero
// - return-to-threshold
// - log domain detector

class SideChain {
protected:
  Average<double> avg;
  DecoupledSmoothedPeakDetector pd;

  float makeUp;
  float samplingRate;
  float inputSmoothDuration;

  double XG;
  double YG;
  float XL;
  float YL;
  float Cdb;

public:
  SideChain() : makeUp(0), inputSmoothDuration(50) {
    setSamplingRate(44100);
    setMakeUp(0);
    setAttack(1);
    setRelease(1);
  }

  ~SideChain() {}

  virtual void setSamplingRate(float sr) {
    samplingRate = sr;
    pd.setSamplingRate(samplingRate);
    avg.resize(static_cast<unsigned long>(inputSmoothDuration * samplingRate * 0.001));
  }

  // virtual void setInputSmoothDuration(float d) {
  //   inputSmoothDuration = d;
  //   avg.resize(static_cast<unsigned long>(inputSmoothDuration * samplingRate * 0.001));
  // }

  virtual void setMakeUp(float m) { makeUp = m; }
  virtual void setAttack(float a) { pd.setAttack(a); }
  virtual void setRelease(float r) { pd.setRelease(r); }

  virtual void setThreshold(float t) = 0;
  virtual void setRatio(float r) = 0;
  virtual void setKnee(float k) = 0;

  virtual sample process(sample in) = 0;
  virtual sample process(sample in, sample m, sample t, sample r, sample k = 0) = 0;
};

//--------------------------- LOG DOMAIN DETECTOR ----------------------------//

class LogDomainSideChain : public SideChain {
private:
  GainComputer<double> gc;

public:
  LogDomainSideChain() : SideChain() {
    setThreshold(0);
    setRatio(1);
    setKnee(0);
  }

  virtual ~LogDomainSideChain() {}

  virtual void setThreshold(float t) { gc.setThreshold(t); }
  virtual void setRatio(float r) { gc.setRatio(r); }
  virtual void setKnee(float k) { gc.setKnee(k); }

  virtual sample process(sample in);
  virtual sample process(sample in, sample m, sample t, sample r, sample k = 0);
};

//=========================== DB ENVELOPE FOLLOWER ===========================//

// This class is designed to be used in dry wet sections of effects where
// differences between the dry and wet output amplitudes are too high (e.g. some
// waveshaper based distortion effects like sgn(x) * (1 - exp(-fabs(x))) which
// are supposed to roughly simulate tube saturation or other hardware).
// It allows to align the wet signal's amplitude to the dry signal's amplitude.
// It takes two input signals and outputs the value by which the second input
// signal should be multiplied to match the amplitude of the first signal.

class LogDomainFlattener {
private:
  Average<sample> avg1, avg2;
  GainComputer<double> gc;
  DecoupledSmoothedPeakDetector pd;

  float makeUp;
  float samplingRate;
  float inputSmoothDuration;

  float thresh;
  float XG;
  float YG;
  float XL;
  float YL;
  float Cdb;

public:
  LogDomainFlattener() : makeUp(0), inputSmoothDuration(50) {
    setSamplingRate(44100);
    setMakeUp(0);
    setAttack(1);
    setRelease(1);
    setRatio(1);
    setKnee(0);
  }
  ~LogDomainFlattener() {}

  void setSamplingRate(float sr) {
    samplingRate = sr;
    pd.setSamplingRate(samplingRate);
    avg1.resize(static_cast<unsigned long>(inputSmoothDuration * samplingRate * 0.001));
    avg2.resize(static_cast<unsigned long>(inputSmoothDuration * samplingRate * 0.001));
  }

  void setMakeUp(float m) { makeUp = m; }
  void setAttack(float a) { pd.setAttack(a); }
  void setRelease(float r) { pd.setRelease(r); }
  void setRatio(float r) { gc.setRatio(r); }
  void setKnee(float k) { gc.setKnee(k); }

  sample process(sample in1, sample in2);
  sample process(sample in1, sample in2, sample m, sample r, sample k = 0);
};

//========================== BASIC COMPRESSOR CLASS ==========================//

// NB : not really tested yet because in pd it is more fun to play directly with
// a child of the SideChain class (aka the [jl/sidechain~] external)

// TODO : create a class using MS encoding / decoding inside for stereo signals
// the side-chain should be fed with the M signal and applied to : M ? L and R ?
// encoding :
// M = 0.5 * (L + R)
// S = 0.5 * (L - R)
// decoding :
// L = M + S
// R = M - S
// source :
// https://www.hackaudio.com/digital-signal-processing/stereo-audio/mid-side-processing/ms-encoding/
// http://www.hackaudio.com/digital-signal-processing/stereo-audio/mid-side-processing/ms-decoding/

enum CompressorDesign {
  FeedforwardCompressorDesign = 0,
  FeedbackCompressorDesign,
  AlternateFeedbackCompressorDesign
};

class Compressor {
private:
  CompressorDesign compressorDesign;
  unsigned int channels;
  float samplingRate;

  LogDomainSideChain sc;

  Ramp<float, sample> rMakeUp;
  Ramp<float, sample> rThreshold;
  Ramp<float, sample> rRatio;
  Ramp<float, sample> rKnee;

  float rampDuration;
  unsigned long rampSamples;

public:
  Compressor(
    unsigned int c = 1,
    CompressorDesign design = FeedforwardCompressorDesign
  ) :
  compressorDesign(design), channels(c),
  samplingRate(44100), rampDuration(10) {
    updateSamplingRate();
  }

  ~Compressor() {}

  void setSamplingRate(float sr);
  void setMakeUp(float m); // ramp
  void setThreshold(float t); // ramp
  void setRatio(float r); // ramp
  void setKnee(float k); // ramp
  void setAttack(float a);
  void setRelease(float r);

  void process(sample **ins, sample **outs, unsigned int blockSize);

private:
  void updateSamplingRate();
};

} /* end namespace jl */

#endif /* _JL_COMPRESS_H_ */
