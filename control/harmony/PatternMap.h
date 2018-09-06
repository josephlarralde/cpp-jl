#ifndef _JL_PATTERNMAP_H_
#define _JL_PATTERNMAP_H_

#define JL_MAX_PATTERN_MAP_LENGTH 64

#include <math.h>
#include "../../jl.h"

namespace jl {

template <typename T>
class PatternMap {
private:
  unsigned int pattern[JL_MAX_PATTERN_MAP_LENGTH];
  unsigned int patternSize;
  unsigned int length;
  float factor;
  T root;

  // tmp calculus variables
  T epsilon;
  T relRoot;
  T rootDiff;
  T relPosition;
  int lowerDegree;
  int upperDegree;
  T ratio;
  T scaled;

  // 0 : factor == 0, 1 : factor == 1, 2 : 0 < factor < 1
  unsigned int factorState;

public:
  PatternMap() :
  patternSize(1), length(1), factor(1), root(0),
  epsilon(1e-9), relRoot(0), rootDiff(0), relPosition(0),
  lowerDegree(0), upperDegree(1),
  ratio(1), scaled(0),
  factorState(1) {
    pattern[0] = 1;
  }

  ~PatternMap() {}

  void setPattern(unsigned int *pat, unsigned int len) {
    patternSize = 0;
    length = static_cast<unsigned int>(fmin(len, JL_MAX_PATTERN_MAP_LENGTH));

    for (unsigned int i = 0; i < length; ++i) {
      patternSize += pat[i];
      pattern[i] = pat[i];
    }

    if (length == 0) {
      patternSize = 1;
      length = 1;
      pattern[0] = 1;
    }
  }

  void setFactor(float f) {
    // factor interpolation feels more linear this way
    factor = pow(1 - (JL_CLIP(f, 0, 1)), 2);

    if (fabs(factor - 1) < epsilon) {
      factorState = 1;
    } else if (fabs(factor) < epsilon) {
      factorState = 0;
    } else {
      factorState = 2;
    }
  }

  void setRoot(T r) {
    root = r;
  }

  // some special cases are optimized :
  // - if factor == 1 : linear, pass value through
  // - if factor == 0 : don't compute curve, simply output closest value

  T process(T in) {
    if (factorState == 1) return in;

    relRoot = root;
    rootDiff = in - relRoot;


    // while (fabs(rootDiff) > patternSize) {
    while (rootDiff > patternSize) {
      relRoot += (rootDiff > 0 ? 1 : -1) * patternSize;
      rootDiff = in - relRoot;
    }

    // if (rootDiff < 0) {
    while (rootDiff < 0) {
      relRoot -= patternSize;
      rootDiff = in - relRoot;
    }

    relPosition = in - relRoot;
    lowerDegree = 0;
    upperDegree = pattern[0];

    for (unsigned int i = 0; i < length; ++i) {
      if (relPosition - upperDegree < 0) {
        break;
      }

      lowerDegree += pattern[i];
      upperDegree += pattern[(i + 1) % length];
    }

    // now scale da shit

    ratio = (relPosition - lowerDegree) / (upperDegree - lowerDegree);

    if (ratio < 0.5) {
      if (factorState == 0) {
        return relRoot + lowerDegree;
      }

      ratio *= 2;
      scaled = 1 - pow(1 - ratio, factor);
      scaled *= 0.5;
    } else {
      if (factorState == 0) {
        return relRoot + upperDegree;
      }

      ratio = (ratio - 0.5) * 2;
      scaled = pow(ratio, factor);
      scaled = (scaled * 0.5) + 0.5;
    }

    scaled *= (upperDegree - lowerDegree);
    return relRoot + lowerDegree + scaled;
  }
};

} /* end namespace jl */

#endif /* _JL_PATTERNMAP_H_ */