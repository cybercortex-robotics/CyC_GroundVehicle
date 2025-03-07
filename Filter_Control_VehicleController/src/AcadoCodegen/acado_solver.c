/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 35 */
real_t state[ 35 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 4];
state[1] = acadoVariables.x[lRun1 * 4 + 1];
state[2] = acadoVariables.x[lRun1 * 4 + 2];
state[3] = acadoVariables.x[lRun1 * 4 + 3];

state[28] = acadoVariables.u[lRun1 * 2];
state[29] = acadoVariables.u[lRun1 * 2 + 1];
state[30] = acadoVariables.od[lRun1 * 5];
state[31] = acadoVariables.od[lRun1 * 5 + 1];
state[32] = acadoVariables.od[lRun1 * 5 + 2];
state[33] = acadoVariables.od[lRun1 * 5 + 3];
state[34] = acadoVariables.od[lRun1 * 5 + 4];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 4] = state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = state[19];

acadoWorkspace.evGu[lRun1 * 8] = state[20];
acadoWorkspace.evGu[lRun1 * 8 + 1] = state[21];
acadoWorkspace.evGu[lRun1 * 8 + 2] = state[22];
acadoWorkspace.evGu[lRun1 * 8 + 3] = state[23];
acadoWorkspace.evGu[lRun1 * 8 + 4] = state[24];
acadoWorkspace.evGu[lRun1 * 8 + 5] = state[25];
acadoWorkspace.evGu[lRun1 * 8 + 6] = state[26];
acadoWorkspace.evGu[lRun1 * 8 + 7] = state[27];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 23. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = ((xd[0])*(xd[0]));
a[1] = (atan((((((real_t)(3.0000000000000000e+00)*od[3])*a[0])+(((real_t)(2.0000000000000000e+00)*od[2])*xd[0]))+od[1])));
a[2] = (((real_t)(0.0000000000000000e+00)-a[1])+xd[2]);
a[3] = (pow(xd[0],3));
a[4] = ((xd[0])*(xd[0]));
a[5] = (((((od[3]*a[3])+(od[2]*a[4]))+(od[1]*xd[0]))+od[0])-xd[1]);
a[6] = ((real_t)(2.0000000000000000e+00)*xd[0]);
a[7] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow((((((real_t)(3.0000000000000000e+00)*od[3])*a[0])+(((real_t)(2.0000000000000000e+00)*od[2])*xd[0]))+od[1]),2))));
a[8] = (((((real_t)(3.0000000000000000e+00)*od[3])*a[6])+((real_t)(2.0000000000000000e+00)*od[2]))*a[7]);
a[9] = ((real_t)(0.0000000000000000e+00)-a[8]);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(1.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = ((real_t)(3.0000000000000000e+00)*((xd[0])*(xd[0])));
a[14] = ((real_t)(2.0000000000000000e+00)*xd[0]);
a[15] = (((od[3]*a[13])+(od[2]*a[14]))+od[1]);
a[16] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[3];
out[1] = a[2];
out[2] = a[5];
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(1.0000000000000000e+00);
out[7] = a[9];
out[8] = a[10];
out[9] = a[11];
out[10] = a[12];
out[11] = a[15];
out[12] = a[16];
out[13] = a[17];
out[14] = a[18];
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = a[19];
out[18] = a[20];
out[19] = a[21];
out[20] = a[22];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 4;
/* Vector of auxiliary variables; number of elements: 19. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = ((xd[0])*(xd[0]));
a[1] = (atan((((((real_t)(3.0000000000000000e+00)*od[3])*a[0])+(((real_t)(2.0000000000000000e+00)*od[2])*xd[0]))+od[1])));
a[2] = (((real_t)(0.0000000000000000e+00)-a[1])+xd[2]);
a[3] = (pow(xd[0],3));
a[4] = ((xd[0])*(xd[0]));
a[5] = (((((od[3]*a[3])+(od[2]*a[4]))+(od[1]*xd[0]))+od[0])-xd[1]);
a[6] = ((real_t)(2.0000000000000000e+00)*xd[0]);
a[7] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+(pow((((((real_t)(3.0000000000000000e+00)*od[3])*a[0])+(((real_t)(2.0000000000000000e+00)*od[2])*xd[0]))+od[1]),2))));
a[8] = (((((real_t)(3.0000000000000000e+00)*od[3])*a[6])+((real_t)(2.0000000000000000e+00)*od[2]))*a[7]);
a[9] = ((real_t)(0.0000000000000000e+00)-a[8]);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(1.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = ((real_t)(3.0000000000000000e+00)*((xd[0])*(xd[0])));
a[14] = ((real_t)(2.0000000000000000e+00)*xd[0]);
a[15] = (((od[3]*a[13])+(od[2]*a[14]))+od[1]);
a[16] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00));
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[3];
out[1] = a[2];
out[2] = a[5];
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(1.0000000000000000e+00);
out[7] = a[9];
out[8] = a[10];
out[9] = a[11];
out[10] = a[12];
out[11] = a[15];
out[12] = a[16];
out[13] = a[17];
out[14] = a[18];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[4]*tmpObjS[3] + tmpFx[8]*tmpObjS[6];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[4]*tmpObjS[4] + tmpFx[8]*tmpObjS[7];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[4]*tmpObjS[5] + tmpFx[8]*tmpObjS[8];
tmpQ2[3] = + tmpFx[1]*tmpObjS[0] + tmpFx[5]*tmpObjS[3] + tmpFx[9]*tmpObjS[6];
tmpQ2[4] = + tmpFx[1]*tmpObjS[1] + tmpFx[5]*tmpObjS[4] + tmpFx[9]*tmpObjS[7];
tmpQ2[5] = + tmpFx[1]*tmpObjS[2] + tmpFx[5]*tmpObjS[5] + tmpFx[9]*tmpObjS[8];
tmpQ2[6] = + tmpFx[2]*tmpObjS[0] + tmpFx[6]*tmpObjS[3] + tmpFx[10]*tmpObjS[6];
tmpQ2[7] = + tmpFx[2]*tmpObjS[1] + tmpFx[6]*tmpObjS[4] + tmpFx[10]*tmpObjS[7];
tmpQ2[8] = + tmpFx[2]*tmpObjS[2] + tmpFx[6]*tmpObjS[5] + tmpFx[10]*tmpObjS[8];
tmpQ2[9] = + tmpFx[3]*tmpObjS[0] + tmpFx[7]*tmpObjS[3] + tmpFx[11]*tmpObjS[6];
tmpQ2[10] = + tmpFx[3]*tmpObjS[1] + tmpFx[7]*tmpObjS[4] + tmpFx[11]*tmpObjS[7];
tmpQ2[11] = + tmpFx[3]*tmpObjS[2] + tmpFx[7]*tmpObjS[5] + tmpFx[11]*tmpObjS[8];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[4] + tmpQ2[2]*tmpFx[8];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[5] + tmpQ2[2]*tmpFx[9];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[6] + tmpQ2[2]*tmpFx[10];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[11];
tmpQ1[4] = + tmpQ2[3]*tmpFx[0] + tmpQ2[4]*tmpFx[4] + tmpQ2[5]*tmpFx[8];
tmpQ1[5] = + tmpQ2[3]*tmpFx[1] + tmpQ2[4]*tmpFx[5] + tmpQ2[5]*tmpFx[9];
tmpQ1[6] = + tmpQ2[3]*tmpFx[2] + tmpQ2[4]*tmpFx[6] + tmpQ2[5]*tmpFx[10];
tmpQ1[7] = + tmpQ2[3]*tmpFx[3] + tmpQ2[4]*tmpFx[7] + tmpQ2[5]*tmpFx[11];
tmpQ1[8] = + tmpQ2[6]*tmpFx[0] + tmpQ2[7]*tmpFx[4] + tmpQ2[8]*tmpFx[8];
tmpQ1[9] = + tmpQ2[6]*tmpFx[1] + tmpQ2[7]*tmpFx[5] + tmpQ2[8]*tmpFx[9];
tmpQ1[10] = + tmpQ2[6]*tmpFx[2] + tmpQ2[7]*tmpFx[6] + tmpQ2[8]*tmpFx[10];
tmpQ1[11] = + tmpQ2[6]*tmpFx[3] + tmpQ2[7]*tmpFx[7] + tmpQ2[8]*tmpFx[11];
tmpQ1[12] = + tmpQ2[9]*tmpFx[0] + tmpQ2[10]*tmpFx[4] + tmpQ2[11]*tmpFx[8];
tmpQ1[13] = + tmpQ2[9]*tmpFx[1] + tmpQ2[10]*tmpFx[5] + tmpQ2[11]*tmpFx[9];
tmpQ1[14] = + tmpQ2[9]*tmpFx[2] + tmpQ2[10]*tmpFx[6] + tmpQ2[11]*tmpFx[10];
tmpQ1[15] = + tmpQ2[9]*tmpFx[3] + tmpQ2[10]*tmpFx[7] + tmpQ2[11]*tmpFx[11];
}

void acado_setObjQN1QN2( real_t* const tmpFx, real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = + tmpFx[0]*tmpObjSEndTerm[0] + tmpFx[4]*tmpObjSEndTerm[3] + tmpFx[8]*tmpObjSEndTerm[6];
tmpQN2[1] = + tmpFx[0]*tmpObjSEndTerm[1] + tmpFx[4]*tmpObjSEndTerm[4] + tmpFx[8]*tmpObjSEndTerm[7];
tmpQN2[2] = + tmpFx[0]*tmpObjSEndTerm[2] + tmpFx[4]*tmpObjSEndTerm[5] + tmpFx[8]*tmpObjSEndTerm[8];
tmpQN2[3] = + tmpFx[1]*tmpObjSEndTerm[0] + tmpFx[5]*tmpObjSEndTerm[3] + tmpFx[9]*tmpObjSEndTerm[6];
tmpQN2[4] = + tmpFx[1]*tmpObjSEndTerm[1] + tmpFx[5]*tmpObjSEndTerm[4] + tmpFx[9]*tmpObjSEndTerm[7];
tmpQN2[5] = + tmpFx[1]*tmpObjSEndTerm[2] + tmpFx[5]*tmpObjSEndTerm[5] + tmpFx[9]*tmpObjSEndTerm[8];
tmpQN2[6] = + tmpFx[2]*tmpObjSEndTerm[0] + tmpFx[6]*tmpObjSEndTerm[3] + tmpFx[10]*tmpObjSEndTerm[6];
tmpQN2[7] = + tmpFx[2]*tmpObjSEndTerm[1] + tmpFx[6]*tmpObjSEndTerm[4] + tmpFx[10]*tmpObjSEndTerm[7];
tmpQN2[8] = + tmpFx[2]*tmpObjSEndTerm[2] + tmpFx[6]*tmpObjSEndTerm[5] + tmpFx[10]*tmpObjSEndTerm[8];
tmpQN2[9] = + tmpFx[3]*tmpObjSEndTerm[0] + tmpFx[7]*tmpObjSEndTerm[3] + tmpFx[11]*tmpObjSEndTerm[6];
tmpQN2[10] = + tmpFx[3]*tmpObjSEndTerm[1] + tmpFx[7]*tmpObjSEndTerm[4] + tmpFx[11]*tmpObjSEndTerm[7];
tmpQN2[11] = + tmpFx[3]*tmpObjSEndTerm[2] + tmpFx[7]*tmpObjSEndTerm[5] + tmpFx[11]*tmpObjSEndTerm[8];
tmpQN1[0] = + tmpQN2[0]*tmpFx[0] + tmpQN2[1]*tmpFx[4] + tmpQN2[2]*tmpFx[8];
tmpQN1[1] = + tmpQN2[0]*tmpFx[1] + tmpQN2[1]*tmpFx[5] + tmpQN2[2]*tmpFx[9];
tmpQN1[2] = + tmpQN2[0]*tmpFx[2] + tmpQN2[1]*tmpFx[6] + tmpQN2[2]*tmpFx[10];
tmpQN1[3] = + tmpQN2[0]*tmpFx[3] + tmpQN2[1]*tmpFx[7] + tmpQN2[2]*tmpFx[11];
tmpQN1[4] = + tmpQN2[3]*tmpFx[0] + tmpQN2[4]*tmpFx[4] + tmpQN2[5]*tmpFx[8];
tmpQN1[5] = + tmpQN2[3]*tmpFx[1] + tmpQN2[4]*tmpFx[5] + tmpQN2[5]*tmpFx[9];
tmpQN1[6] = + tmpQN2[3]*tmpFx[2] + tmpQN2[4]*tmpFx[6] + tmpQN2[5]*tmpFx[10];
tmpQN1[7] = + tmpQN2[3]*tmpFx[3] + tmpQN2[4]*tmpFx[7] + tmpQN2[5]*tmpFx[11];
tmpQN1[8] = + tmpQN2[6]*tmpFx[0] + tmpQN2[7]*tmpFx[4] + tmpQN2[8]*tmpFx[8];
tmpQN1[9] = + tmpQN2[6]*tmpFx[1] + tmpQN2[7]*tmpFx[5] + tmpQN2[8]*tmpFx[9];
tmpQN1[10] = + tmpQN2[6]*tmpFx[2] + tmpQN2[7]*tmpFx[6] + tmpQN2[8]*tmpFx[10];
tmpQN1[11] = + tmpQN2[6]*tmpFx[3] + tmpQN2[7]*tmpFx[7] + tmpQN2[8]*tmpFx[11];
tmpQN1[12] = + tmpQN2[9]*tmpFx[0] + tmpQN2[10]*tmpFx[4] + tmpQN2[11]*tmpFx[8];
tmpQN1[13] = + tmpQN2[9]*tmpFx[1] + tmpQN2[10]*tmpFx[5] + tmpQN2[11]*tmpFx[9];
tmpQN1[14] = + tmpQN2[9]*tmpFx[2] + tmpQN2[10]*tmpFx[6] + tmpQN2[11]*tmpFx[10];
tmpQN1[15] = + tmpQN2[9]*tmpFx[3] + tmpQN2[10]*tmpFx[7] + tmpQN2[11]*tmpFx[11];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 5];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 5 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 5 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 5 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 5 + 4];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 3 ]), &(acadoVariables.W[ runObj * 9 ]), &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 12 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[40];
acadoWorkspace.objValueIn[1] = acadoVariables.x[41];
acadoWorkspace.objValueIn[2] = acadoVariables.x[42];
acadoWorkspace.objValueIn[3] = acadoVariables.x[43];
acadoWorkspace.objValueIn[4] = acadoVariables.od[50];
acadoWorkspace.objValueIn[5] = acadoVariables.od[51];
acadoWorkspace.objValueIn[6] = acadoVariables.od[52];
acadoWorkspace.objValueIn[7] = acadoVariables.od[53];
acadoWorkspace.objValueIn[8] = acadoVariables.od[54];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( &(acadoWorkspace.objValueOut[ 3 ]), acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7];
Gu2[2] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[6]*Gu1[4] + Gx1[7]*Gu1[6];
Gu2[3] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[5] + Gx1[7]*Gu1[7];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7];
Gu2[6] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_multBTW1_R1( real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 42] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6];
acadoWorkspace.H[iRow * 42 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7];
acadoWorkspace.H[iRow * 42 + 20] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6];
acadoWorkspace.H[iRow * 42 + 21] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[4]*Gu1[2] + Gx1[8]*Gu1[4] + Gx1[12]*Gu1[6];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[4]*Gu1[3] + Gx1[8]*Gu1[5] + Gx1[12]*Gu1[7];
Gu2[2] = + Gx1[1]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[9]*Gu1[4] + Gx1[13]*Gu1[6];
Gu2[3] = + Gx1[1]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[9]*Gu1[5] + Gx1[13]*Gu1[7];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[14]*Gu1[6];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[14]*Gu1[7];
Gu2[6] = + Gx1[3]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[11]*Gu1[4] + Gx1[15]*Gu1[6];
Gu2[7] = + Gx1[3]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[11]*Gu1[5] + Gx1[15]*Gu1[7];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[2] + Q11[2]*Gu1[4] + Q11[3]*Gu1[6] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[3] + Q11[2]*Gu1[5] + Q11[3]*Gu1[7] + Gu2[1];
Gu3[2] = + Q11[4]*Gu1[0] + Q11[5]*Gu1[2] + Q11[6]*Gu1[4] + Q11[7]*Gu1[6] + Gu2[2];
Gu3[3] = + Q11[4]*Gu1[1] + Q11[5]*Gu1[3] + Q11[6]*Gu1[5] + Q11[7]*Gu1[7] + Gu2[3];
Gu3[4] = + Q11[8]*Gu1[0] + Q11[9]*Gu1[2] + Q11[10]*Gu1[4] + Q11[11]*Gu1[6] + Gu2[4];
Gu3[5] = + Q11[8]*Gu1[1] + Q11[9]*Gu1[3] + Q11[10]*Gu1[5] + Q11[11]*Gu1[7] + Gu2[5];
Gu3[6] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[2] + Q11[14]*Gu1[4] + Q11[15]*Gu1[6] + Gu2[6];
Gu3[7] = + Q11[12]*Gu1[1] + Q11[13]*Gu1[3] + Q11[14]*Gu1[5] + Q11[15]*Gu1[7] + Gu2[7];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[4]*w11[1] + Gx1[8]*w11[2] + Gx1[12]*w11[3] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[5]*w11[1] + Gx1[9]*w11[2] + Gx1[13]*w11[3] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[6]*w11[1] + Gx1[10]*w11[2] + Gx1[14]*w11[3] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[7]*w11[1] + Gx1[11]*w11[2] + Gx1[15]*w11[3] + w12[3];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + w12[0];
w13[1] = + Q11[4]*w11[0] + Q11[5]*w11[1] + Q11[6]*w11[2] + Q11[7]*w11[3] + w12[1];
w13[2] = + Q11[8]*w11[0] + Q11[9]*w11[1] + Q11[10]*w11[2] + Q11[11]*w11[3] + w12[2];
w13[3] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + w12[3];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3];
w12[1] += + Gx1[4]*w11[0] + Gx1[5]*w11[1] + Gx1[6]*w11[2] + Gx1[7]*w11[3];
w12[2] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3];
w12[3] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3];
w12[1] += + Gx1[4]*w11[0] + Gx1[5]*w11[1] + Gx1[6]*w11[2] + Gx1[7]*w11[3];
w12[2] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3];
w12[3] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1];
w12[1] += + Gu1[2]*U1[0] + Gu1[3]*U1[1];
w12[2] += + Gu1[4]*U1[0] + Gu1[5]*U1[1];
w12[3] += + Gu1[6]*U1[0] + Gu1[7]*U1[1];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = 0.0;
;
RDy1[1] = 0.0;
;
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2];
QDy1[1] = + Q2[3]*Dy1[0] + Q2[4]*Dy1[1] + Q2[5]*Dy1[2];
QDy1[2] = + Q2[6]*Dy1[0] + Q2[7]*Dy1[1] + Q2[8]*Dy1[2];
QDy1[3] = + Q2[9]*Dy1[0] + Q2[10]*Dy1[1] + Q2[11]*Dy1[2];
}

void acado_condensePrep(  )
{
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 8 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.E[ 16 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 24 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 32 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.E[ 40 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 56 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.E[ 64 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.E[ 72 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 72 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 64 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 56 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.W1, 7, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 48 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 6, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 40 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 5, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 32 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 24 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.W1, 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 16 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 8 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.E[ 80 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 88 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 104 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.E[ 112 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 128 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.E[ 136 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.E[ 144 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 144 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 136 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 128 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.W1, 7, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 120 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 6, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 112 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 5, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 104 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 96 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.W1, 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 88 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 80 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 152 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 168 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 176 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.E[ 184 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 200 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.E[ 208 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 208 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 200 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 192 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.W1, 7, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 184 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 6, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 176 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 5, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 168 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.W1, 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 152 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 224 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.E[ 232 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.E[ 232 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 248 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 248 ]), &(acadoWorkspace.E[ 256 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.E[ 264 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 264 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 256 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 248 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.W1, 7, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 6, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 232 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 5, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 224 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 216 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.E[ 272 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.E[ 280 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 296 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 296 ]), &(acadoWorkspace.E[ 304 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.E[ 312 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 312 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 304 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 296 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.W1, 7, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 288 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 6, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 280 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 5, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 272 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4 );

/* Column: 5 */
acado_moveGuE( &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.E[ 320 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 328 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.E[ 328 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 344 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 344 ]), &(acadoWorkspace.E[ 352 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 352 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 344 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 336 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.W1, 7, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 328 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 6, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 320 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 5 );

/* Column: 6 */
acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 368 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.E[ 376 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 376 ]), &(acadoWorkspace.E[ 384 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 384 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 376 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 368 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.W1, 7, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 360 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 6 );

/* Column: 7 */
acado_moveGuE( &(acadoWorkspace.evGu[ 56 ]), &(acadoWorkspace.E[ 392 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 392 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 408 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 408 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 392 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.W1, 7 );

/* Column: 8 */
acado_moveGuE( &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.E[ 416 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.E[ 424 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 424 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 416 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 8 );

/* Column: 9 */
acado_moveGuE( &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.E[ 432 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 432 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 9 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );

acadoWorkspace.sbar[4] = acadoWorkspace.d[0];
acadoWorkspace.sbar[5] = acadoWorkspace.d[1];
acadoWorkspace.sbar[6] = acadoWorkspace.d[2];
acadoWorkspace.sbar[7] = acadoWorkspace.d[3];
acadoWorkspace.sbar[8] = acadoWorkspace.d[4];
acadoWorkspace.sbar[9] = acadoWorkspace.d[5];
acadoWorkspace.sbar[10] = acadoWorkspace.d[6];
acadoWorkspace.sbar[11] = acadoWorkspace.d[7];
acadoWorkspace.sbar[12] = acadoWorkspace.d[8];
acadoWorkspace.sbar[13] = acadoWorkspace.d[9];
acadoWorkspace.sbar[14] = acadoWorkspace.d[10];
acadoWorkspace.sbar[15] = acadoWorkspace.d[11];
acadoWorkspace.sbar[16] = acadoWorkspace.d[12];
acadoWorkspace.sbar[17] = acadoWorkspace.d[13];
acadoWorkspace.sbar[18] = acadoWorkspace.d[14];
acadoWorkspace.sbar[19] = acadoWorkspace.d[15];
acadoWorkspace.sbar[20] = acadoWorkspace.d[16];
acadoWorkspace.sbar[21] = acadoWorkspace.d[17];
acadoWorkspace.sbar[22] = acadoWorkspace.d[18];
acadoWorkspace.sbar[23] = acadoWorkspace.d[19];
acadoWorkspace.sbar[24] = acadoWorkspace.d[20];
acadoWorkspace.sbar[25] = acadoWorkspace.d[21];
acadoWorkspace.sbar[26] = acadoWorkspace.d[22];
acadoWorkspace.sbar[27] = acadoWorkspace.d[23];
acadoWorkspace.sbar[28] = acadoWorkspace.d[24];
acadoWorkspace.sbar[29] = acadoWorkspace.d[25];
acadoWorkspace.sbar[30] = acadoWorkspace.d[26];
acadoWorkspace.sbar[31] = acadoWorkspace.d[27];
acadoWorkspace.sbar[32] = acadoWorkspace.d[28];
acadoWorkspace.sbar[33] = acadoWorkspace.d[29];
acadoWorkspace.sbar[34] = acadoWorkspace.d[30];
acadoWorkspace.sbar[35] = acadoWorkspace.d[31];
acadoWorkspace.sbar[36] = acadoWorkspace.d[32];
acadoWorkspace.sbar[37] = acadoWorkspace.d[33];
acadoWorkspace.sbar[38] = acadoWorkspace.d[34];
acadoWorkspace.sbar[39] = acadoWorkspace.d[35];
acadoWorkspace.sbar[40] = acadoWorkspace.d[36];
acadoWorkspace.sbar[41] = acadoWorkspace.d[37];
acadoWorkspace.sbar[42] = acadoWorkspace.d[38];
acadoWorkspace.sbar[43] = acadoWorkspace.d[39];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 18 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 12 ]), &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 24 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 72 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 84 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 96 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 36 ]) );

acadoWorkspace.QDy[40] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[41] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[42] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[43] = + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[2];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 40 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[40] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[41] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[42] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[43] + acadoWorkspace.QDy[40];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[40] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[41] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[42] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[43] + acadoWorkspace.QDy[41];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[40] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[41] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[42] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[43] + acadoWorkspace.QDy[42];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[40] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[41] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[42] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[43] + acadoWorkspace.QDy[43];
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.d[0];
acadoWorkspace.sbar[5] = acadoWorkspace.d[1];
acadoWorkspace.sbar[6] = acadoWorkspace.d[2];
acadoWorkspace.sbar[7] = acadoWorkspace.d[3];
acadoWorkspace.sbar[8] = acadoWorkspace.d[4];
acadoWorkspace.sbar[9] = acadoWorkspace.d[5];
acadoWorkspace.sbar[10] = acadoWorkspace.d[6];
acadoWorkspace.sbar[11] = acadoWorkspace.d[7];
acadoWorkspace.sbar[12] = acadoWorkspace.d[8];
acadoWorkspace.sbar[13] = acadoWorkspace.d[9];
acadoWorkspace.sbar[14] = acadoWorkspace.d[10];
acadoWorkspace.sbar[15] = acadoWorkspace.d[11];
acadoWorkspace.sbar[16] = acadoWorkspace.d[12];
acadoWorkspace.sbar[17] = acadoWorkspace.d[13];
acadoWorkspace.sbar[18] = acadoWorkspace.d[14];
acadoWorkspace.sbar[19] = acadoWorkspace.d[15];
acadoWorkspace.sbar[20] = acadoWorkspace.d[16];
acadoWorkspace.sbar[21] = acadoWorkspace.d[17];
acadoWorkspace.sbar[22] = acadoWorkspace.d[18];
acadoWorkspace.sbar[23] = acadoWorkspace.d[19];
acadoWorkspace.sbar[24] = acadoWorkspace.d[20];
acadoWorkspace.sbar[25] = acadoWorkspace.d[21];
acadoWorkspace.sbar[26] = acadoWorkspace.d[22];
acadoWorkspace.sbar[27] = acadoWorkspace.d[23];
acadoWorkspace.sbar[28] = acadoWorkspace.d[24];
acadoWorkspace.sbar[29] = acadoWorkspace.d[25];
acadoWorkspace.sbar[30] = acadoWorkspace.d[26];
acadoWorkspace.sbar[31] = acadoWorkspace.d[27];
acadoWorkspace.sbar[32] = acadoWorkspace.d[28];
acadoWorkspace.sbar[33] = acadoWorkspace.d[29];
acadoWorkspace.sbar[34] = acadoWorkspace.d[30];
acadoWorkspace.sbar[35] = acadoWorkspace.d[31];
acadoWorkspace.sbar[36] = acadoWorkspace.d[32];
acadoWorkspace.sbar[37] = acadoWorkspace.d[33];
acadoWorkspace.sbar[38] = acadoWorkspace.d[34];
acadoWorkspace.sbar[39] = acadoWorkspace.d[35];
acadoWorkspace.sbar[40] = acadoWorkspace.d[36];
acadoWorkspace.sbar[41] = acadoWorkspace.d[37];
acadoWorkspace.sbar[42] = acadoWorkspace.d[38];
acadoWorkspace.sbar[43] = acadoWorkspace.d[39];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.evGu[ 56 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 40 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -5.0000000000000000e-01;
acadoVariables.lbValues[1] = -4.2999999999999999e-01;
acadoVariables.lbValues[2] = -5.0000000000000000e-01;
acadoVariables.lbValues[3] = -4.2999999999999999e-01;
acadoVariables.lbValues[4] = -5.0000000000000000e-01;
acadoVariables.lbValues[5] = -4.2999999999999999e-01;
acadoVariables.lbValues[6] = -5.0000000000000000e-01;
acadoVariables.lbValues[7] = -4.2999999999999999e-01;
acadoVariables.lbValues[8] = -5.0000000000000000e-01;
acadoVariables.lbValues[9] = -4.2999999999999999e-01;
acadoVariables.lbValues[10] = -5.0000000000000000e-01;
acadoVariables.lbValues[11] = -4.2999999999999999e-01;
acadoVariables.lbValues[12] = -5.0000000000000000e-01;
acadoVariables.lbValues[13] = -4.2999999999999999e-01;
acadoVariables.lbValues[14] = -5.0000000000000000e-01;
acadoVariables.lbValues[15] = -4.2999999999999999e-01;
acadoVariables.lbValues[16] = -5.0000000000000000e-01;
acadoVariables.lbValues[17] = -4.2999999999999999e-01;
acadoVariables.lbValues[18] = -5.0000000000000000e-01;
acadoVariables.lbValues[19] = -4.2999999999999999e-01;
acadoVariables.ubValues[0] = 5.0000000000000000e-01;
acadoVariables.ubValues[1] = 4.2999999999999999e-01;
acadoVariables.ubValues[2] = 5.0000000000000000e-01;
acadoVariables.ubValues[3] = 4.2999999999999999e-01;
acadoVariables.ubValues[4] = 5.0000000000000000e-01;
acadoVariables.ubValues[5] = 4.2999999999999999e-01;
acadoVariables.ubValues[6] = 5.0000000000000000e-01;
acadoVariables.ubValues[7] = 4.2999999999999999e-01;
acadoVariables.ubValues[8] = 5.0000000000000000e-01;
acadoVariables.ubValues[9] = 4.2999999999999999e-01;
acadoVariables.ubValues[10] = 5.0000000000000000e-01;
acadoVariables.ubValues[11] = 4.2999999999999999e-01;
acadoVariables.ubValues[12] = 5.0000000000000000e-01;
acadoVariables.ubValues[13] = 4.2999999999999999e-01;
acadoVariables.ubValues[14] = 5.0000000000000000e-01;
acadoVariables.ubValues[15] = 4.2999999999999999e-01;
acadoVariables.ubValues[16] = 5.0000000000000000e-01;
acadoVariables.ubValues[17] = 4.2999999999999999e-01;
acadoVariables.ubValues[18] = 5.0000000000000000e-01;
acadoVariables.ubValues[19] = 4.2999999999999999e-01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
state[0] = acadoVariables.x[index * 4];
state[1] = acadoVariables.x[index * 4 + 1];
state[2] = acadoVariables.x[index * 4 + 2];
state[3] = acadoVariables.x[index * 4 + 3];
state[28] = acadoVariables.u[index * 2];
state[29] = acadoVariables.u[index * 2 + 1];
state[30] = acadoVariables.od[index * 5];
state[31] = acadoVariables.od[index * 5 + 1];
state[32] = acadoVariables.od[index * 5 + 2];
state[33] = acadoVariables.od[index * 5 + 3];
state[34] = acadoVariables.od[index * 5 + 4];

acado_integrate(state, index == 0);

acadoVariables.x[index * 4 + 4] = state[0];
acadoVariables.x[index * 4 + 5] = state[1];
acadoVariables.x[index * 4 + 6] = state[2];
acadoVariables.x[index * 4 + 7] = state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[40] = xEnd[0];
acadoVariables.x[41] = xEnd[1];
acadoVariables.x[42] = xEnd[2];
acadoVariables.x[43] = xEnd[3];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[40];
state[1] = acadoVariables.x[41];
state[2] = acadoVariables.x[42];
state[3] = acadoVariables.x[43];
if (uEnd != 0)
{
state[28] = uEnd[0];
state[29] = uEnd[1];
}
else
{
state[28] = acadoVariables.u[18];
state[29] = acadoVariables.u[19];
}
state[30] = acadoVariables.od[50];
state[31] = acadoVariables.od[51];
state[32] = acadoVariables.od[52];
state[33] = acadoVariables.od[53];
state[34] = acadoVariables.od[54];

acado_integrate(state, 1);

acadoVariables.x[40] = state[0];
acadoVariables.x[41] = state[1];
acadoVariables.x[42] = state[2];
acadoVariables.x[43] = state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 3 */
real_t tmpDy[ 3 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 5];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 5 + 4];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[40];
acadoWorkspace.objValueIn[1] = acadoVariables.x[41];
acadoWorkspace.objValueIn[2] = acadoVariables.x[42];
acadoWorkspace.objValueIn[3] = acadoVariables.x[43];
acadoWorkspace.objValueIn[4] = acadoVariables.od[50];
acadoWorkspace.objValueIn[5] = acadoVariables.od[51];
acadoWorkspace.objValueIn[6] = acadoVariables.od[52];
acadoWorkspace.objValueIn[7] = acadoVariables.od[53];
acadoWorkspace.objValueIn[8] = acadoVariables.od[54];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 3] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 6];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9 + 1] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 4] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 7];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9 + 2] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 5] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 8];
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

