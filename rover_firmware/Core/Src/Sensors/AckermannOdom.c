
/* Include Files */
#include <math.h>
#include <string.h>
#include "Sensors/AckermannOdom.h"
/* Variable Definitions */
static double X[3];
static double P[9];

/* Function Definitions */

/*
 * Arguments    : double v
 *                double phi
 * Return Type  : void
 */


void AckermannOdom(double v, double phi, float speedVar, float steeringVar,
                   float *w, float M[4])
{
  double w_tmp_tmp;
  double v_w;
  double vcos;
  double vcos_tmp;
  double vsin;
  double G[9];
  double d;
  double T[4];
  double V[6];
  int i;
  int i1;
  double b_V[6];
  int i2;
  double b_G[9];
  double c_G[9];


  w_tmp_tmp = tan(phi);
  *w = v * w_tmp_tmp / 0.305;

  /* prediction: */
  v_w = v / *w;
  if ((*w > 0.0001) || (fabs(v_w) < 10000.0)) {
    vcos_tmp = X[2] + *w * 0.02;
    vcos = -v_w * (cos(vcos_tmp) - cos(X[2]));
    vsin = -v_w * (sin(vcos_tmp) - sin(X[2]));
    G[0] = 1.0;
    G[3] = 0.0;
    G[6] = -vcos;
    G[1] = 0.0;
    G[4] = 1.0;
    G[7] = -vsin;
    G[2] = 0.0;
    G[5] = 0.0;
    G[8] = 1.0;
    T[0] = 1.0;
    T[2] = 0.0;
    T[1] = w_tmp_tmp / 0.305;
    T[3] = v * (w_tmp_tmp * w_tmp_tmp + 1.0) / 0.305;
    V[0] = -vsin / v;
    vcos_tmp = X[2] + *w * 0.02;
    V[3] = vsin / *w + v_w * 0.02 * cos(vcos_tmp);
    V[1] = vcos / v;
    V[4] = -vcos / *w + v_w * 0.02 * sin(vcos_tmp);
    for (i = 0; i < 2; i++) {
      vcos_tmp = T[i + 2];
      v_w = T[i] * speedVar + vcos_tmp * 0.0;
      vcos_tmp = T[i] * 0.0 + vcos_tmp * steeringVar;
      M[i] = v_w + vcos_tmp * 0.0;
      M[i + 2] = v_w * T[1] + vcos_tmp * T[3];
      V[3 * i + 2] = 0.02 * (double)i;
    }

    vcos_tmp = X[0];
    v_w = X[1];
    d = X[2];
    X[0] = vcos_tmp + -vsin;
    X[1] = v_w + vcos;
    X[2] = d + *w * 0.02;
    for (i = 0; i < 3; i++) {
      i1 = (int)G[i + 3];
      vcos_tmp = G[i + 6];
      for (i2 = 0; i2 < 3; i2++) {
        c_G[i + 3 * i2] = ((double)(int)G[i] * P[3 * i2] + (double)i1 * P[3 * i2
                           + 1]) + vcos_tmp * P[3 * i2 + 2];
      }

      vcos_tmp = V[i + 3];
      b_V[i] = V[i] * M[0] + vcos_tmp * M[1];
      b_V[i + 3] = V[i] * M[2] + vcos_tmp * M[3];
      vcos_tmp = c_G[i + 3];
      v_w = c_G[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        b_G[i + 3 * i1] = (c_G[i] * G[i1] + vcos_tmp * G[i1 + 3]) + v_w * G[i1 +
          6];
      }
    }

    for (i = 0; i < 3; i++) {
      vcos_tmp = b_V[i + 3];
      for (i1 = 0; i1 < 3; i1++) {
        G[i + 3 * i1] = b_V[i] * V[i1] + vcos_tmp * V[i1 + 3];
      }
    }

    for (i = 0; i < 9; i++) {
      P[i] = b_G[i] + G[i];
    }
  } else {
    vcos = v * cos(X[2]) * 0.02;
    vsin = v * sin(X[2]) * 0.02;
    vcos_tmp = X[0];
    v_w = X[1];
    d = X[2];
    X[0] = vcos_tmp + vcos;
    X[1] = v_w + vsin;
    X[2] = d + *w * 0.02;
    G[0] = 1.0;
    G[3] = 0.0;
    G[6] = -vsin;
    G[1] = 0.0;
    G[4] = 1.0;
    G[7] = vcos;
    G[2] = 0.0;
    G[5] = 0.0;
    G[8] = 1.0;
    V[0] = vcos / v;
    V[3] = 0.0;
    V[1] = vsin / v;
    V[4] = 0.0;
    V[2] = 0.0;
    T[0] = 1.0;
    V[5] = 0.02;
    T[2] = 0.0;
    T[1] = tan(phi) / 0.305;
    T[3] = v * (w_tmp_tmp * w_tmp_tmp + 1.0) / 0.305;
    for (i = 0; i < 2; i++) {
      vcos_tmp = T[i + 2];
      v_w = T[i] * speedVar + vcos_tmp * 0.0;
      vcos_tmp = T[i] * 0.0 + vcos_tmp * steeringVar;
      M[i] = v_w + vcos_tmp * 0.0;
      M[i + 2] = v_w * T[1] + vcos_tmp * T[3];
    }

    for (i = 0; i < 3; i++) {
      i1 = (int)G[i + 3];
      vcos_tmp = G[i + 6];
      for (i2 = 0; i2 < 3; i2++) {
        c_G[i + 3 * i2] = ((double)(int)G[i] * P[3 * i2] + (double)i1 * P[3 * i2
                           + 1]) + vcos_tmp * P[3 * i2 + 2];
      }

      vcos_tmp = V[i + 3];
      b_V[i] = V[i] * M[0] + vcos_tmp * M[1];
      b_V[i + 3] = V[i] * M[2] + vcos_tmp * M[3];
      vcos_tmp = c_G[i + 3];
      v_w = c_G[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        b_G[i + 3 * i1] = (c_G[i] * G[i1] + vcos_tmp * G[i1 + 3]) + v_w * G[i1 +
          6];
      }
    }

    for (i = 0; i < 3; i++) {
      vcos_tmp = b_V[i + 3];
      for (i1 = 0; i1 < 3; i1++) {
        G[i + 3 * i1] = b_V[i] * V[i1] + vcos_tmp * V[i1 + 3];
      }
    }

    for (i = 0; i < 9; i++) {
      P[i] = b_G[i] + G[i];
    }
  }
}


/*
 * Arguments    : void
 * Return Type  : void
 */
void AckermannOdom_init(void)
{
  X[0] = 0.0;
  X[1] = 0.0;
  X[2] = 0.0;
  memset(&P[0], 0, 9U * sizeof(double));
}

void Ackermann_RetrieveOdom(ROVER_STATE* state){
	state->x=(float)X[0];
	state->y=(float)X[1];
	state->theta=(float)X[2];
	state->covariance[0]=(float)P[0];
	state->covariance[1]=(float)P[3];
	state->covariance[2]=(float)P[6];
	state->covariance[3]=(float)P[1];
	state->covariance[4]=(float)P[4];
	state->covariance[5]=(float)P[7];
	state->covariance[6]=(float)P[2];
	state->covariance[7]=(float)P[5];
	state->covariance[8]=(float)P[8];
}
