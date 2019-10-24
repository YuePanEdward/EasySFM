// Ceres Unit Vector Bundle Adjuster.
// Dec 13 2012
//
// Author: michal.havlena@geod.baug.ethz.ch (Michal Havlena)
//
// MATLAB MEX wrapper.
//
// Depends on Ceres Solver.
// http://code.google.com/p/ceres-solver/

#include "mex.h"

#include "uvw_problem.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  mwSize n, m, nc, np, nz, n1, m1, nz1, n2, m2, nz2, n3, m3, nz3;
  mwIndex *ir1, *ir2, *ir3, *jc1, *jc2, *jc3;
  double *prc, *prp, *pr1, *pr2, *pr3, *prc2, *prp2;
  mwIndex i, j, k;
  mxArray *gfpr;

  int num_cameras, num_points, num_observations, num_parameters;
  int *point_index, *camera_index;
  double *observations, *parameters;

  bool geodesic = false;
  bool sparse = false;
  bool verbose = false;
  int niter = 500;
  int nconstframes = 0;
  int nconstpoints = 0;

  if (nrhs < 5) {
    mexErrMsgTxt("Five input arguments required.");
  } else if (nlhs > 2) {
    mexErrMsgTxt("Too many output arguments.");
  }

  if (mxIsSparse(prhs[0]) || !mxIsDouble(prhs[0]))
    mexErrMsgTxt("Double matrix expected as first parameter!");
  if (mxIsSparse(prhs[1]) || !mxIsDouble(prhs[1]))
    mexErrMsgTxt("Double matrix expected as second parameter!");
  if (!mxIsSparse(prhs[2]) || !mxIsDouble(prhs[2]))
    mexErrMsgTxt("Sparse double matrix 'u' expected as third parameter!");
  if (!mxIsSparse(prhs[3]) || !mxIsDouble(prhs[3]))
    mexErrMsgTxt("Sparse double matrix 'v' expected as fourth parameter!");
  if (!mxIsSparse(prhs[4]) || !mxIsDouble(prhs[4]))
    mexErrMsgTxt("Sparse double matrix 'w' expected as fifth parameter!");

  if (nrhs > 5) {
    if (mxIsSparse(prhs[5]) || !mxIsStruct(prhs[5]))
      mexErrMsgTxt("Struct 'opt' expected as sixth parameter!");
    m = mxGetM(prhs[5]);
    n = mxGetN(prhs[5]);
    if (n*m != 1)
      mexErrMsgTxt("Sixth parameter should be [1 x 1]!");
    gfpr = mxGetField(prhs[5], 0, "geodesic");
    if (gfpr) {
      if (!mxIsLogical(gfpr))
        mexErrMsgTxt("'geodesic' should be logical!");
      geodesic = mxGetScalar(gfpr);
    }
    gfpr = mxGetField(prhs[5], 0, "sparse");
    if (gfpr) {
      if (!mxIsLogical(gfpr))
        mexErrMsgTxt("'sparse' should be logical!");
      sparse = mxGetScalar(gfpr);
    }
    gfpr = mxGetField(prhs[5], 0, "verbose");
    if (gfpr) {
      if (!mxIsLogical(gfpr))
        mexErrMsgTxt("'verbose' should be logical!");
      verbose = mxGetScalar(gfpr);
    }
    gfpr = mxGetField(prhs[5], 0, "niter");
    if (gfpr) {
      if (!mxIsInt32(gfpr))
        mexErrMsgTxt("'niter' should be int32!");
      niter = mxGetScalar(gfpr);
    }
    gfpr = mxGetField(prhs[5], 0, "nconstframes");
    if (gfpr) {
      if (!mxIsInt32(gfpr))
        mexErrMsgTxt("'nconstframes' should be int32!");
      nconstframes = mxGetScalar(gfpr);
    }
    gfpr = mxGetField(prhs[5], 0, "nconstpoints");
    if (gfpr) {
      if (!mxIsInt32(gfpr))
        mexErrMsgTxt("'nconstpoints' should be int32!");
      nconstpoints = mxGetScalar(gfpr);
    }
  }

  m = mxGetM(prhs[0]);
  nc = mxGetN(prhs[0]);
  prc = mxGetPr(prhs[0]);

  if (m != 6)
    mexErrMsgTxt("First parameter should be [6 x ncams]!");

  m = mxGetM(prhs[1]);
  np = mxGetN(prhs[1]);
  prp = mxGetPr(prhs[1]);

  if (m != 3)
    mexErrMsgTxt("Second parameter should be [3 x npoints]!");

  m = mxGetM(prhs[2]);
  n = mxGetN(prhs[2]);
  nz = mxGetNzmax(prhs[2]);
  pr1 = mxGetPr(prhs[2]);
  ir1 = mxGetIr(prhs[2]);
  jc1 = mxGetJc(prhs[2]);

  for (j = n; (j > 0) && (jc1[j] == jc1[j-1]); --j);
  n1 = j;
  nz1 = jc1[j];
  m1 = 0;
  for (; j > 0; --j) {
    if ((jc1[j] > 0) && (ir1[jc1[j]-1]+1 > m1)) m1 = ir1[jc1[j]-1]+1;
  }

  m = mxGetM(prhs[3]);
  n = mxGetN(prhs[3]);
  nz = mxGetNzmax(prhs[3]);
  pr2 = mxGetPr(prhs[3]);
  ir2 = mxGetIr(prhs[3]);
  jc2 = mxGetJc(prhs[3]);

  for (j = n; (j > 0) && (jc2[j] == jc2[j-1]); --j);
  n2 = j;
  nz2 = jc2[j];
  m2 = 0;
  for (; j > 0; --j) {
    if ((jc2[j] > 0) && (ir2[jc2[j]-1]+1 > m2)) m2 = ir2[jc2[j]-1]+1;
  }

  m = mxGetM(prhs[4]);
  n = mxGetN(prhs[4]);
  nz = mxGetNzmax(prhs[4]);
  pr3 = mxGetPr(prhs[4]);
  ir3 = mxGetIr(prhs[4]);
  jc3 = mxGetJc(prhs[4]);

  for (j = n; (j > 0) && (jc3[j] == jc3[j-1]); --j);
  n3 = j;
  nz3 = jc3[j];
  m3 = 0;
  for (; j > 0; --j) {
    if ((jc3[j] > 0) && (ir3[jc3[j]-1]+1 > m3)) m3 = ir3[jc3[j]-1]+1;
  }

  if ((m1 != m2) || (m1 != m3) || (n1 != n2) || (n1 != n3) || (nz1 != nz2) || (nz1 != nz3))
    mexErrMsgTxt("Inconsistent sizes of observation matrices!");

  if (m1 > nc)
    mexErrMsgTxt("Missing cameras to some observations!");

  if (n1 > np)
    mexErrMsgTxt("Missing points to some observations!");

  for (j = 0; j < n1+1; ++j) {
    if ((jc1[j] != jc2[j]) || (jc1[j] != jc3[j]))
      mexErrMsgTxt("Inconsistent columns of observation matrices!");
  }

  for (i = 0; i < nz1; ++i) {
    if ((ir1[i] != ir2[i]) || (ir1[i] != ir3[i]))
      mexErrMsgTxt("Inconsistent rows of observation matrices!");
  }

  num_cameras = nc;
  num_points = np;
  num_observations = nz1;

  camera_index = new int[num_observations];
  point_index = new int[num_observations];
  observations = new double[3 * num_observations];

  k = 0;
  for (j = 0; j < n1; ++j) {
    for (i = 0; i < jc1[j+1]-jc1[j]; ++i) {
      camera_index[k] = ir1[k];
      point_index[k] = j;
      observations[3*k] = pr1[k];
      observations[3*k + 1] = pr2[k];
      observations[3*k + 2] = pr3[k];
      ++k;
    }
  }

  num_parameters = 6 * num_cameras + 3 * num_points;
  parameters = new double[num_parameters];

  for (i = 0; i < 6*nc; ++i) {
    parameters[i] = prc[i];
  }

  for (i = 0; i < 3*np; ++i) {
    parameters[6*num_cameras + i] = prp[i];
  }

  uvbac::UVWProblem uvw_problem(num_cameras, num_points, num_observations, num_parameters, camera_index, point_index, observations, parameters);
  mexPrintf("%s\n", uvw_problem.Optimize(geodesic, sparse, verbose, niter, nconstframes, nconstpoints).c_str());

  plhs[0] = mxCreateDoubleMatrix(6, nc, mxREAL);
  prc2 = mxGetPr(plhs[0]);

  plhs[1] = mxCreateDoubleMatrix(3, np, mxREAL);
  prp2 = mxGetPr(plhs[1]);

  for (i = 0; i < 6*nc; ++i) {
    prc2[i] = parameters[i];
  }

  for (i = 0; i < 3*np; ++i) {
    prp2[i] = parameters[6*num_cameras + i];
  }

  delete[] parameters;
  delete[] observations;
  delete[] point_index;
  delete[] camera_index;
}
