#pragma once
#ifndef stateDerivative_h
#define stateDerivative_h
#include "macro.h"
#include <math.h>

// camera
#define sigX 0.05 // [m]
#define sigY 0.05 // [m]
#define sigZ1 0

// sonar
#define sigZ2 0.05 // [m]

// imu 1 (MPU)
#define sigP1 0
#define sigQ1 0
#define sigR1 0

#define sigAX1 0
#define sigAY1 0
#define sigAZ1 0

// imu 2 (LSM)
#define sigP2 0
#define sigQ2 0
#define sigR2 0

#define sigAX2 0
#define sigAY2 0
#define sigAZ2 0

// define physical properties

#define g 9.81  // [m/sec^2]
#define m 0.654  // [kg]
#define Ixx 0.06615 // [kg*m^2]
#define Iyy 0.06641 // [kg*m^2]
#define Izz 0.007310 // [kg*m^2]
#define d 0.1061 // [m]
#define P_by_omega2 0.00000196032
#define Qb_by_omega2 0.000005
#define T_by_omega2 0.000005

void stateDerivative(float* x, float* xDot, float* a, float* powerCmd) {
	// calculate motor torques
	float Qb[4] = {};
	for (int i = 0; i < 4; i++) {
		Qb[i] = Qb_by_omega2 / P_by_omega2 * powerCmd[i];
	}

	// calculate thrusts (defined positive)
	float T[4] = {};
	for (int i = 0; i < 4; i++) {
		T[i] = T_by_omega2 / P_by_omega2 * powerCmd[i];
	}

	// calculate propeller caused forces and moments
	float Fz = -T[0] - T[1] - T[2] - T[3];
	float Mk = -T[0] * d + T[1] * d + T[2] * d - T[3] * d;
	float Mm = +T[0] * d - T[1] * d + T[2] * d - T[3] * d;
	float Mn = +Qb[0] + Qb[1] - Qb[2] - Qb[3];

	// pre-calculate trig expressions
	float cphi = cos(x[PHI]); float sphi = sin(x[PHI]);
	float cthe = cos(x[THE]); float sthe = sin(x[THE]);
	float cpsi = cos(x[PSI]); float spsi = sin(x[PSI]);

	// calculate state vector derivative
	xDot[X] = cthe * cpsi * x[U] + (-cphi * spsi + sphi * sthe * cpsi) * x[V] + (sphi * spsi + cphi * sthe * cpsi) * x[W];
	xDot[Y] = cthe * spsi * x[U] + (cphi * cpsi + sphi * sthe * spsi) * x[V] + (-sphi * cpsi + cphi * sthe * spsi) * x[W];
	xDot[Z] = -sthe * x[U] + sphi * cthe * x[V] + cphi * cthe * x[W];
	xDot[PHI] = x[P] + (x[Q] * sphi + x[R] * cphi) * tan(x[THE]);
	xDot[THE] = x[Q] * cphi - x[R] * sphi;
	xDot[PSI] = (x[Q] * sphi + x[R] * cphi) / cthe;
	xDot[U] = -g * sthe + x[R] * x[V] - x[Q] * x[W];
	xDot[V] = g * sphi * cthe - x[R] * x[U] + x[P] * x[W];
	xDot[W] = Fz / m + g * cphi * cthe + x[Q] * x[U] - x[P] * x[V]; 
	xDot[P] = 1 / Ixx * (Mk + (Iyy - Izz) * x[Q] * x[R]);
	xDot[Q] = 1 / Iyy * (Mm + (Izz - Ixx) * x[P] * x[R]);
	xDot[R] = 1 / Izz * (Mn + (Ixx - Iyy) * x[P] * x[Q]);

	a[X] = 0;
	a[Y] = 0;
	a[Z] = Fz / m;
}


#endif // !"stateDerivative.h"