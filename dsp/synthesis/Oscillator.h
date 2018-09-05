/**
 * @file Oscillator.h
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief oscillators
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

#ifndef _JL_OSCILLATOR_H_
#define _JL_OSCILLATOR_H_

// changing this value doesn't seem to have any effect on sinusoid aliasing :|
// Maybe it has something to do with the fact that we always use powers of 2 ... ?
#define JL_OSCILLATOR_DEFAULT_WAVETABLE_SIZE 512

#include "../../jl.h"

namespace jl {

enum OscControl {
  OscControlFreq = 0,
  OscControlPhase
};

enum OscShape {
  OscShapeSin = 0,
  OscShapeHann,
  OscShapeTri,
  OscShapeSaw,
  OscShapeSqr,
  OscShapeTriAA,
  OscShapeSawAA,
  OscShapeSqrAA
};

//============================== BASE CLASS ==================================//

class Oscillator {
protected:
  float sr;
  float freq;
  float phase; // normalized phase

public:
  Oscillator() :
  sr(44100), phase(0) {}

  ~Oscillator() {}

  virtual void setControlMethod(OscControl c) = 0;
  virtual void setSamplingRate(float sr) = 0;

  // virtual void setFrequency(float f) = 0;
  // virtual void setPhase(float f) = 0;
  virtual void process(sample *in, sample *out, unsigned int blockSize) = 0;
};

//=============================== DERIVED ====================================//

class WavetableOscillator : public Oscillator {
private:
  OscShape shape;
  OscControl controlMethod;
  unsigned int tableSize;
  sample *table;

public:
  WavetableOscillator(OscShape s = OscShapeSin,
                      OscControl c = OscControlFreq,
                      unsigned int ts = JL_OSCILLATOR_DEFAULT_WAVETABLE_SIZE) :
  Oscillator(),
  shape(s), controlMethod(c), tableSize(ts) {
    table = new sample[tableSize];
    createTable();
  }

  virtual ~WavetableOscillator() {
    delete [] table;
  }

  virtual void setControlMethod(OscControl c);
  virtual void setSamplingRate(float sr);

  // virtual void setFrequency(float f);
  // virtual void setPhase(float f);
  virtual void process(sample *in, sample *out, unsigned int blockSize);

private:
  void createTable();
};

// TODO : PHASE MODULATION SYNTHESIS !!! (remember gen patch ?)

} /* end namespace jl */

#endif /* _JL_OSCILLATOR_H_ */