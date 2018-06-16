/**
 * @file Gbend.cpp
 * @author Joseph Larralde
 * @date 07/06/2018
 * @brief pitch signal controlled sample player with integrated fade in / fade out system and start / stop messages
 *
 * This class is a monophonic (multi-channel but not polyphonic) audio player
 * with various useful features :
 * - pitch signal control input in relative semitone values
 * - start and stop messages
 * - integrated fade in / fade out system
 * - special interrupt fade before new fade in if start is called while still playing
 * - additionnal beg, end (begin / end bounds in the buffer in ms), loop, pitch
 *   and reverse parameters, taken into account at start or end playing time
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

// #define GBEND_MAX(X, M) (X < M) ? M : X
// #define GBEND_CLIP(X, A, B) (X < A) ? A : ((X > B) ? B : X)

#include "../jl.dsp.h"
#include "../utilities/wavetable.h"
#include "Gbend.h"
// #include "m_pd.h"

void
Gbend::setBuffer(float *b, unsigned long bLen, float bSr, unsigned int bChannels) {
  // TODO: add a #DEFINE and #ifdef condition to allow building a dumb version
  // where the buffer is updated immediately

  // buf = b;
  // bufLen = bLen;
  // bufMsr = bSr / 1000;
  // bufChannels = bChannels;

  nextBuf = b;
  nextBufLen = bLen;
  nextBufMsr = bSr / 1000;
  nextBufChannels = bChannels;

  switchBufAsap = true;
}

void
Gbend::setSamplingRate(float sr) {
  msr = sr / 1000;
}

void
Gbend::setBegin(float f) {
  begin = f;
}

void
Gbend::setEnd(float f) {
  end = f;
}

void
Gbend::setPitch(float f) {
  cpitch = JL_CLIP(f, -120, 120);
}

void
Gbend::setRvs(bool b) {
  rvs = b;
}

void
Gbend::setLoop(bool b) {
  loop = realLoop = b;
}

void
Gbend::setFadeIn(float f) {
  fadi = JL_MAX(f, 1);
}

void
Gbend::setFadeOut(float f) {
  fado = JL_MAX(f, 1);
}

void
Gbend::setFades(float f) {
  fadi = fado = JL_MAX(f, 1);
}

void
Gbend::setInterrupt(float f) {
  interrupt = JL_MAX(f, 1);
}

void
Gbend::start() {
  if (!interrupting) {
    interrupting = true;
    interruptIndex = 0;
    realPlaying = true;

    if (!playing) {
      playing = true;
      silentBlockEnd = true;

      // if trueLoop has been set false by stop msg,
      // we set it back to true here :
      if (loop) {
        realLoop = true;
      }
    }
  }
}

void
Gbend::stop() {
  if(!stopping) {
    stopping = true;
    realStopping = true;
    realLoop = false;
  }
}

void
Gbend::process(float *in, float **outs, unsigned int blockSize) {
  long longBlockSize = (long)(blockSize);

  if (blk != longBlockSize) {
    blk = longBlockSize;
    // todo : fill sIndexArray with zeroes
    // or something to avoid potential segfaults ?
  }

  if (buf == 0) {
    playSilence(outs);
    updateBuffer();
    computeParameters();
    return;
  }

  long ntmp = blk;

  long index;
  long maxIndex = bufLen - 3;
  double relIndex, relEnvIndex;
  double iVal; // interrupt fade value (normalized)
  double fVal; // actual fade (in or out) value (normalized)
  double frac; // decimal part of the sample index to compute

  double *sIndexArrayCursor = &(sIndexArray[0]);

  if (playing) {
    while (ntmp-- > 0) {
      *sIndexArrayCursor = exp((*in++ + pitch) * 0.057762265) +
                              ((ntmp == blk - 1)
                                ? lastLastIndex
                                : *(sIndexArrayCursor - 1));

      // isFirstVector var is set to true at the same time as playing var
      if (isFirstVector) {

        // INDEX SIGNAL GETS 1 SAMPLE DELAYED FOR CONTINUITY
        *sIndexArrayCursor = lastLastIndex = 0;
        isFirstVector = false;

        // I've been debugging with pd :

        // post("");
        // post("trig !");
        // post("computed parameters :");
        // post("blk : %i", blk);
        // post("msr : %f", msr);
        // post("sBegin : %i", sBegin);
        // post("sLen : %i", sLen);
        // post("sFadi : %i", sFadi);
        // post("sFado : %i", sFado);
        // post("sInterrupt : %i", sInterrupt);
        // post("pitch : %f", pitch);
      }

      if (ntmp == 0) { // LAST SIG VECT VAL
        lastLastIndex = *sIndexArrayCursor;
      }

      // we compute sStopLen once and for all as soon as we get the stop message
      if (stopping) {
        sStopLen = ((long) *sIndexArrayCursor + sFado < sStopLen)
                 ? (long) (*sIndexArrayCursor + sFado)
                 : sStopLen;

        stopping = false;
      }

      // READ SAMPLES AND FADES ACCORDING TO THE SIGNAL INDEX ARRAY + BILINEAR INTERPOLATION

      //_______ CHECK IF REVERSE _____________________________________________//

      if (playing) {
        if (!realRvs) {
          relIndex = relEnvIndex = *sIndexArrayCursor; // the index according to the beginning of selection
        } else {
          relIndex = sLen - 1 - *sIndexArrayCursor; // the same in reverse
          relEnvIndex = sStopLen - 1 - *sIndexArrayCursor;
        }
      } else {
        relIndex = relEnvIndex = 0;
      }

      //_______ FIRST SET INTERRUPT FADE VALUE _______________________________//

      if (interrupting) {
        if (interruptIndex < sInterrupt) {
          iVal = silentBlockEnd ? 0 : (1 - (double) interruptIndex / (sInterrupt - 1));
          interruptIndex++;
        } else {
          iVal = 0;
        }
      } else {
        iVal = 1;
      }

      //_______ THEN SET CURRENT FADE IN OR OUT VALUE ________________________//

      // TODO : compute surrounding 4 points and interpolate ?
      // worth it or not ?

      if (relEnvIndex < sFadi) {
        if (relEnvIndex < 0) {
          fVal = 0;
        } else {
          fVal = relEnvIndex / sFadi;
        }
      } else if (relEnvIndex > sStopLen - sFado) {
        if (relEnvIndex > sStopLen) {
          fVal = 0;
        } else {
          fVal = (sStopLen - relEnvIndex) / sFado;
        }
      } else {
        fVal = 1;
      }

      //======================= NOW USE LOWER LEVEL CODE =====================//

      index = floor(sBegin + relIndex); // rounded down

      if (index < 1) {
        frac = 0;
        index = 1;
      } else if (index > maxIndex) {
        frac = 1;
        index = maxIndex;
      } else {
        frac = (sBegin + relIndex) - index;
      }

      // this is the lower level code

      // @todo : implement channel offset as in the to-be-updated max gbend external
      // fp = buf + ((ch + x->a_channel_offset) % channelcount) + index * channelcount;

      float res[bufChannels];

      // this is a paranoid safety guard to avoid segfaults if buffer was
      // updated by some obscure thread :

      if (index < bufLen - 3 && buf != 0) {
        // ok, go (this works with pd but other conditions might be required for more safety)
        interpolateBicubicStride(buf, &(res[0]), index, frac, bufChannels);
      } else {
        // don't read into buf !
        for (unsigned int i = 0; i < bufChannels; ++i) {
          res[i] = 0;
        }
      }

      int currentIndex = blk - ntmp - 1;

      for (unsigned int ch = 0; ch < channels; ch++) {
        float *out = outs[ch];
        if (ch >= bufChannels) {
          out[currentIndex] = outs[ch - bufChannels][currentIndex];
        } else {
          out[currentIndex] = res[ch] * iVal * fVal;
        }
      }

      //_______ HERE WE KNOW IF WE'LL BE READING TOO FAR : ___________________//

      if (playing && !interrupting && (relEnvIndex > sStopLen || relEnvIndex < 0)) { // we're reading too far ...
        if (realLoop) { // ... so we loop (never after stop msg received)
          computeParameters();
          isFirstVector = true;
          endReachCallback(2);
        } else {
          playing = false; // ... so we stop
          iVal = 0;

          if (!realStopping) {
            endReachCallback(1);
          } else {
            realStopping = false;
            endReachCallback(0);
          }
        }
      }

      sIndexArrayCursor++;

      //_________THIS_IS_THE_ONLY_GOOD_PLACE_TO_TRIG_SAMPLE_PLAYING___________//

      // AN EXCEPTION :
      // in case of end playing and retrig during the same signal vector
      // we use the variable realPlaying set true by bang()
      // because playing might have been set to false in this loop

      if (!interrupting && realPlaying) {
        updateBuffer();
        computeParameters();

        playing = true;
        realPlaying = false;
      }

      // MOST GENERAL CASE :

      if (interrupting && (interruptIndex >= sInterrupt)) { // interrupt is just finished
        interrupting = false;
        interruptIndex = 0;
        silentBlockEnd = false;
        isFirstVector = true;

        updateBuffer();

        if (playing) {
          computeParameters();
        }
      }
    } // end of (while(ntmp--))
  } else { // NOT PLAYING
    playSilence(outs);
    updateBuffer();
    computeParameters();
  }
}

void
Gbend::endReachCallback(int endReachType) {
  // do nothing in base class
}

void
Gbend::bufUpdatedCallback() {
  // do nothing in base class
}

//================================ PRIVATE ===================================//

void
Gbend::computeParameters() {
  if (bufLen > 0) {
    double tmp;

    if (end < begin) {
      tmp = end;
    } else {
      tmp = begin;
    }

    if ((end < begin && !rvs) || (begin < end && rvs)) {
      realRvs = true;
    } else {
      realRvs = false;
    }

    // MINIMUM SELECTION 1 SIGNAL BLOCK SIZE
    // TODO: CHECK IF MINIMUM 2 SAMPLES WOULD WORK AS WELL
    // (ALLOWING 1 SAMPLE FADES)

    sBegin = (long) (tmp * bufMsr);
    sBegin = (long) ((sBegin < 0)
                    ? 0
                    : ((sBegin > bufLen - blk)
                      ? (bufLen - blk)
                      : sBegin));

    sLen = (long) (fabs(end - begin) * bufMsr);
    sLen = (sLen > blk) ? sLen : blk;
    sStopLen = sLen;

    sFadi = (long) (fadi * msr);
    sFado = (long) (fado * msr);
    sInterrupt = (long) (interrupt * msr);

    if (sFadi + sFado >= sLen) {
      if (sFadi < sLen / 2) {
        sFado = sLen - sFadi;
      } else if (sFado < sLen / 2) {
        sFadi = sLen - sFado;
      } else {
        sFadi = sFado = sLen / 2;
      }
    }

    if (sInterrupt > sLen - sFadi) {
      sInterrupt = sLen - sFadi;
    }

    pitch = cpitch;
  }
}

void
Gbend::updateBuffer() {
  //*
  if (switchBufAsap) {
    buf = nextBuf;
    bufLen = nextBufLen;
    bufMsr = nextBufMsr;
    bufChannels = nextBufChannels;
    switchBufAsap = false;

    // post("Gbend using new buffer of length %i, msr %f and %i channels", bufLen, bufMsr, bufChannels);
    bufUpdatedCallback();
  }
  //*/
}

void Gbend::playSilence(float **outs) {
  for (unsigned int ch = 0; ch < channels; ch++) {
    float *out = outs[ch];
    int ntmp = blk;

    while (ntmp-- > 0) {
      *out++ = 0.;
    }
  }
}
