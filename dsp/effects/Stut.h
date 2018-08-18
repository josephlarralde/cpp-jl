/**
 * @file Stut.h
 * @author Joseph Larralde
 * @date 09/06/2018
 * @brief pitch signal controlled stutter effect with integrated fade in / fade out system and start / stop messages
 *
 * Stutter effect featuring :
 *
 * It is intended to be simple enough to allow an easy integration into a variety
 * of environments
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

#ifndef _JL_STUT_H_
#define _JL_STUT_H_

#include "../../jl.h"

namespace jl {

class Stut {
private:
  unsigned int channels;
  float sr;
  float msr;

  float slice;
  float fadi;
  float fado;
  float interrupt;
  float release;
  long sSlice;
  long sFadi;
  long sFado;
  long sStopLen;
  long sInterrupt;
  bool interrupting;
  long interruptIndex;
  long sRelease;
  bool releasing;
  long releaseIndex;

  bool playing, realPlaying;
  bool stopping, realStopping;
  int loops, nextLoops;
  int loopCounter;
  bool silentBlockEnd;

  long index;

  float *buffer;
  float bufferDuration;
  unsigned long bufferSamples;
  bool recording;
  unsigned long recordIndex;

  long blk;

public:
  Stut(float bDuration, unsigned int c = 1) :
  channels(c),
  msr(44.1), slice(0),
  fadi(5), fado(5), interrupt(5), release(5),
  sSlice(0), sStopLen(0),
  interrupting(false), interruptIndex(0),
  releasing(false), releaseIndex(0),
  playing(false), realPlaying(false), stopping(false), realStopping(false),
  loops(1), nextLoops(1), loopCounter(0), silentBlockEnd(false),
  index(0), recording(false), recordIndex(0),
  blk(64) {
    bufferDuration = bDuration;
    bufferSamples = (unsigned long) (msr * bufferDuration);
    buffer = new float[bufferSamples * channels];

    sFadi = fadi * msr;
    sFado = fado * msr;
    sInterrupt = interrupt * msr;
    sRelease = release * msr;
  }

  ~Stut() {
    delete [] buffer;
  }

  void setSamplingRate(float f);

  void setSliceDuration(float f);
  void setSlices(long n);
  void setFades(float f);
  void setFadeIn(float f);
  void setFadeOut(float f);
  void setInterrupt(float f);
  void setRelease(float f);

  void start();
  void stop();

  // this method will be called from the audio thread
  void process(float **ins, float **outs, unsigned int blockSize);

  // this is called during process
  // override it in child classes to get useful sample accurate events
  virtual void endReachCallback(int endReachType);

private:
  void computeParameters();
};

} /* end namespace jl */

#endif /* _JL_STUT_H_ */