/**
 * @file Tonnetz.h
 * @author Joseph Larralde
 * @date 14/08/2018
 * @brief generate scale patterns by navigating through tonnetz using P/R/L transitions
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

// see
// http://www.oxfordhandbooks.com/view/10.1093/oxfordhb/9780199935321.001.0001/oxfordhb-9780199935321-e-003
// and
// https://guichaoua.gitlab.io/web-hexachord/
// for more ideas

#ifndef _JL_TONNETZ_H_
#define _JL_TONNETZ_H_

#define JL_TONNETZ_DEFAULT_QUAD 0 // natural scale patterns
#define JL_TONNETZ_DEFAULT_PENTA 0 // natural scale patterns
#define JL_TONNETZ_DEFAULT_SCALE 0 // natural scale patterns

#include <vector>
// #include <cmath>
#include <cstdlib>

#ifdef major
/* urgh: glibc defines 'major'... */
#undef major
#endif

namespace jl {

enum TonnetzDirection {
  TonnetzDirectionLeft = 0,
  TonnetzDirectionRight,
  TonnetzDirectionBackwards
};

enum TonnetzTransition {
  TonnetzTransitionP = 0, // Parallel / Picarde
  TonnetzTransitionR, // Relative
  TonnetzTransitionL // Leittonwechsel (?) / LeadingToneExchange
};

// what seem to be the right terms, explained in this very good video :
// https://www.youtube.com/watch?v=-RRJdy0yriw

// Leittonwechsel : "[ ... ] où la fondamentale d’un accord majeur baisse d’un
// demi-ton vers sa sensible pour générer un accord mineur dont la fondamentale
// est située une tierce majeure au-dessus de celle de l’accord majeur."
// or also seen in litterature :
// L <=> "Leading tone exchange"

enum TonnetzMode {
  TonnetzModeDuad = 0,
  TonnetzModeTriad,
  TonnetzModeQuad,
  TonnetzModePenta,
  TonnetzModeScale,
  TonnetzModeInterval
};

class Tonnetz {
private:
  int root;
  bool major;
  TonnetzMode mode;
  TonnetzTransition previousTransition;

  const char *chordNames[12] = {
    "C", "C#", "D", "Eb", "E", "F", "F#", "G", "Ab", "A", "Bb", "B"
  };

  std::vector<unsigned int> duad = { 7, 5 };

  std::vector< std::vector<unsigned int> > triads = {{ 4, 3, 5 }, { 3, 4, 5 }};

  std::vector< std::vector< std::vector<unsigned int> > > quads = {{
    { 4, 3, 4, 1 }, { 3, 4, 3, 2 } // resp. major and minor seventh following natural scale pattern
  }, {
    { 4, 3, 4, 1 }, { 3, 4, 4, 1 } // major seventh
  }, {
    { 4, 3, 3, 2 }, { 3, 4, 3, 2 } // minor seventh
  }};

  std::vector< std::vector< std::vector<unsigned int> > > pentas = {{
    { 4, 1, 2, 4, 1 }, { 3, 2, 2, 3, 2 } // resp. major and minor seventh following natural scale pattern
  }, {
    { 2, 3, 2, 2, 3 }, { 3, 2, 2, 3, 2 } // same pattern but no seventh in major
  }, {
    { 4, 1, 2, 4, 1 }, { 2, 1, 4, 1, 4 } // same pattern but no seventh in minor
  }};

  // add "hexas" as kind of bluenote-augmented pentas ?

  std::vector< std::vector< std::vector<unsigned int> > > scales = {{
    { 2, 2, 1, 2, 2, 2, 1 }, { 2, 1, 2, 2, 1, 2, 2 }
  }, {
    { 2, 2, 1, 2, 2, 2, 1 }, { 2, 1, 2, 2, 1, 3, 1 } // melodic ?
  }};
  // todo: do something for harmonic / melodic minor scales ?

  std::vector<unsigned int> interval = { 1 };

  // with this we can get transitions[TonnetzDirectionLeft][<previous_TonnetzTransition>]
  // to apply the right transition while moving using "directional arrows" on 3 directions
  std::vector< std::vector<TonnetzTransition> > transitions = {
    { // left
      TonnetzTransitionL, // P
      TonnetzTransitionP, // R
      TonnetzTransitionR  // L
    },
    { // right
      TonnetzTransitionR, // P
      TonnetzTransitionL, // R
      TonnetzTransitionP  // L
    },
    { // backwards
      TonnetzTransitionP, // P
      TonnetzTransitionR, // R
      TonnetzTransitionL  // L
    }
  };

public:
  Tonnetz() :
    // C major is the default tonality
    root(0),
    major(true),
    mode(TonnetzModeTriad),
    previousTransition(TonnetzTransitionP) {}

  ~Tonnetz() {}

  void setRoot(int r) {
    root = r;
    normalizeRoot();
  }

  void setMajor(bool m) {
    major = m;
  }

  void setInterval(unsigned int i) {
    interval[0] = i;
  }

  void setTonnetzMode(TonnetzMode tm) {
    mode = tm;
  }

  void transpose(int t) {
    root += t;
    normalizeRoot();
  }

  void applyDirection(TonnetzDirection td) {
    // TODO (see cosmic-composer script above)
    applyTransition(transitions[td][previousTransition]);
  }

  void applyTransition(TonnetzTransition tt) {
    switch (tt) {
      case TonnetzTransitionP:
        // nothing to do
        break;
      case TonnetzTransitionR:
        root += (major ? 9 : 3);
        break;
      // relative de dominante ou sous-dominante secondaire
      case TonnetzTransitionL:
        root += (major ? 4 : 8);
        break;
      // this never happens
      default:
        break;
    }
    
    root %= 12;
    major = !major; // majority switches on each transition
    previousTransition = tt;
  } 

  std::vector<unsigned int> &getPattern() {
    switch (mode) {
      case TonnetzModeDuad:
        return duad;
        break;
      case TonnetzModeTriad:
        return triads[major ? 0 : 1];
        break;
      case TonnetzModeQuad:
        return quads[JL_TONNETZ_DEFAULT_QUAD][major ? 0 : 1];
        break;
      case TonnetzModePenta:
        return pentas[JL_TONNETZ_DEFAULT_PENTA][major ? 0 : 1];
        break;
      case TonnetzModeScale:
        return scales[JL_TONNETZ_DEFAULT_SCALE][major ? 0 : 1];
        break;
      case TonnetzModeInterval:
        return interval;
        break;
    }
  }

  int getRoot() {
    return root;
  }

private:
  void normalizeRoot() {
    if (root < 0) {
      root = root + 12 * ((std::abs(root) / 12) + 1);
    }
    root %= 12;
  }
};

} /* end namespace jl */

#endif /* _JL_TONNETZ_H_ */
