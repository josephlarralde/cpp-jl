#ifndef _JL_H_
#define _JL_H_

#define JL_MAX(X, M) (X < M) ? M : X
#define JL_CLIP(X, A, B) (X < A) ? A : ((X > B) ? B : X)

#endif /* _JL_H_ */