#include <stdlib.h>

#include "mex.h"
#include "matrix.h"

#include "ray2pose.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  double *d, *p, p2[3*3], *q, *hs, *ohs;
  int i, j, k, num_sol, odim[3];
  mwSize mdc;

  /* check number of arguments */
  if ((nrhs > 3) || (nrhs < 2))
    mexErrMsgTxt("Wrong number of input arguments.");
  else if (nlhs > 1)
    mexErrMsgTxt("Too many output arguments.");

  /* import data and prepare array for solutions */
  mdc = mxGetNumberOfDimensions(prhs[0]);
  if (mdc != 2)
    mexErrMsgTxt("First input argument should be a 2D matrix.");
  else if (((mxGetDimensions(prhs[0]))[0] !=3) || ((mxGetDimensions(prhs[0]))[1] != 3))
      mexErrMsgTxt("First input argument matrix should be 3x3.");
    else q = (double *)mxGetPr(prhs[0]);

  mdc = mxGetNumberOfDimensions(prhs[1]);
  if (mdc != 2)
    mexErrMsgTxt("Second input argument should be a 2D matrix.");
  else if (((mxGetDimensions(prhs[1]))[0] !=3) || ((mxGetDimensions(prhs[1]))[1] != 3))
      mexErrMsgTxt("Second input argument matrix should be 3x3.");
    else d = (double *)mxGetPr(prhs[1]);

  if (nrhs == 3) {
    mdc = mxGetNumberOfDimensions(prhs[2]);
    if (mdc != 2)
      mexErrMsgTxt("Third input argument should be a 2D matrix.");
    else if (((mxGetDimensions(prhs[2]))[0] !=3) || ((mxGetDimensions(prhs[2]))[1] != 3))
        mexErrMsgTxt("Third input argument matrix should be 3x3.");
      else p = (double *)mxGetPr(prhs[2]);
  } else {
    for (i=0; i<3*3; ++i) {
      p2[i] = 0;
    }
    p = p2;
  }

  hs = (double *)malloc(4*4*8*sizeof(double));

  /* call function */
  num_sol = ray2pose(d,p,q,hs);

  /* export solutions and transpose them */
  odim[0] = 3; odim[1] = 4; odim[2] = num_sol;
  plhs[0] = mxCreateNumericArray(3,odim,mxDOUBLE_CLASS,mxREAL);
  ohs = (double *)mxGetPr(plhs[0]);

  for (i=0; i<num_sol; ++i) {
    for (j=0; j<4; ++j) {
      for (k=0; k<3; ++k) {
        *ohs++ = *hs;
        hs += 4;
      }
      hs -= 11;
    }
    hs += 12;
  }

  hs -= 4*4*num_sol;
  free(hs);

  return;
}
