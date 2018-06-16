/**
 * @file Gbend.h
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

#ifndef _JL_GBEND_H_
#define _JL_GBEND_H_

#define JL_GBEND_MAX_BLOCK_SIZE 4096

class Gbend {
private:
  unsigned int channels;
  float msr; // samples / ms
  float pitch, cpitch;
  float fadi; // ms
  long sFadi; // samples
  float fado; //ms
  long sFado; // samples
  float interrupt; // ms
  long sInterrupt; //samples
  bool interrupting;
  bool playing, realPlaying; // see why 2 vars are needed in Gbend.cpp line 329
  bool stopping, realStopping;

  double interruptIndex;
  double sIndexArray[JL_GBEND_MAX_BLOCK_SIZE];
  double lastLastIndex;
  bool isFirstVector;

  float begin, end;
  long sBegin, sLen, sStopLen;
  bool silentBlockEnd;

  bool rvs, realRvs;
  bool loop, realLoop;

  // BE CAREFUL WITH THESE VARIABLES :

  // TRY TO NOT DELETE ANY REFERENCE TO THE PREVIOUS BUFFER BEFORE THE bufUpdateCallback
  // HAS BEEN CALLED IN THE DERIVED CLASS YOU SHOULD BE USING.
  // THEN SWAP BUFFERS IN YOUR HOST ENVIRONMENT.
  // SEE THE PD EXTERNAL FOR AN EXAMPLE IMPLEMENTATION.

  // SOME SEGFAULTS AND VIOLENT CRASHES STILL HAPPEN SOMETIMES ... NEED TO DIG THIS
  // PROBABLY A THREAD SAFETY ISSUE

  float *buf;
  float bufMsr;
  unsigned int bufChannels;
  long bufLen; // samples

  float *nextBuf;
  float nextBufMsr;
  unsigned int nextBufChannels;
  long nextBufLen; // samples

  bool switchBufAsap;

  long blk;

public:
  Gbend(unsigned int c = 1) :
  channels(c),
  msr(44.1), pitch(0), cpitch(0), fadi(5), fado(5), interrupt(5),
  interrupting(false), playing(false), realPlaying(false), stopping(false), realStopping(false),
  interruptIndex(0), lastLastIndex(0), isFirstVector(true),
  begin(0), end(0), sBegin(0), sLen(0), sStopLen(0), silentBlockEnd(false),
  rvs(false), realRvs(false), loop(false), realLoop(false),
  bufMsr(1), bufLen(0),
  nextBufMsr(1), nextBufLen(0), switchBufAsap(false),
  blk(64) {
    sFadi = fadi * msr;
    sFado = fado * msr;
    sInterrupt = interrupt * msr;
  }

  ~Gbend() {}

  void setBuffer(float *buf, unsigned long length = 0, float sr = 1, unsigned int channels = 1);
  void setSamplingRate(float sr);

  void setBegin(float f);
  void setEnd(float f);
  void setPitch(float f);
  void setRvs(bool b);
  void setLoop(bool b);
  void setFadeIn(float f);
  void setFadeOut(float f);
  void setFades(float f);
  void setInterrupt(float f);

  void start();
  void stop();

  // this method will be called from the audio thread
  // the input signal is the pitch control signal
  void process(float *in, float **outs, unsigned int blockSize);

  // this is called during process
  // override it in child classes to get useful sample accurate events
  virtual void endReachCallback(int endReachType);
  virtual void bufUpdatedCallback();

private:
  void computeParameters();
  void updateBuffer();
  void playSilence(float **outs);
};

#endif /* _JL_GBEND_H_ */