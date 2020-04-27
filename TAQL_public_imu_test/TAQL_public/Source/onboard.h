#pragma once
#ifndef onboard_h
#define onboard_h
#include "stateDerivative.h"

// From the AccelGyroMag.cpp example
#include "../Navio/Common/MPU9250.h"
#include "../Navio/Navio2/LSM9DS1.h"
#include "../Navio/Common/Util.h"
#include <unistd.h>
#include <string>
#include <memory>


using namespace Navio;

//Auto Ptr copying from Dr.Johnson's code 
/*
std::unique_ptr <InertialSensor> get_inertial_sensor() {

	#ifdef IMU_MPU9250
		auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
		return ptr;
	#endif
	#ifdef IMU_LSM9DS1
		auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
		return ptr;
	#endif

}
*/

void matMult(float a[12][12], float* b, float* mul);
void matMult(float a[12][12], float b[12][12], float mul[12][12]);
void matMult5(float a[12][12], float b[12][3], float mul[12][3]);
void matMult4(float a[12][3], float b[3][12], float mul[12][12]);
void matMult3(float a[12][3], float b[3][3], float mul[12][3]);
void matMult2(float a[3][12], float* b, float* mul);
void matMult1(float a[3][12], float b[12][3], float mul[3][3]);
void matMult0(float a[3][12], float b[12][12], float mul[3][12]);

void matScale(float a[12][12], float con, float mul[12][12]);
void matScale(float* a, float con, float* mul);

void matAdd(float a[12][12], float b[12][12], float mul[12][12]);
void matAdd(float a[3][3], float b[3][3], float mul[3][3]);

void matSub(float* a, float* b, float* mul);
void matInv(float a[3][3], float mul[3][3]);


void vehicleController(float* stick, float* measData, float* powerCmd) {
	float Ks[3] = { 3.14, 3.14, 3.14 }; // Ks is a feedforward gain
	float Kp[3] = { .1, .1, .1 }; // Kp is a proportional feedback gain

	float motorCmd[4] = {};

	float pqrFiltered[3] = {};
	for (int i = 0; i < 3; i++) {
		pqrFiltered[i] = (measData[P1 + i] + measData[P2 + i]) / float(2); // simply avg the two different IMUs together. DOES NOT account for biases
	}

	for (int i = 0; i < 3; i ++) {
		motorCmd[i] = Kp[i]*(Ks[i]*stick[i] - pqrFiltered[i]);
	}
	motorCmd[3] = stick[THRUST];

	powerCmd[0] = -motorCmd[0] + motorCmd[1] + motorCmd[2] + motorCmd[3];
	powerCmd[1] =  motorCmd[0] - motorCmd[1] + motorCmd[2] + motorCmd[3];
	powerCmd[2] =  motorCmd[0] + motorCmd[1] - motorCmd[2] + motorCmd[3];
	powerCmd[3] = -motorCmd[0] - motorCmd[1] - motorCmd[2] + motorCmd[3];

	for (int i = 0; i < 4; i++) {
		if (motorCmd[i] > 1) motorCmd[i] = 1;
		if (motorCmd[i] < 0) motorCmd[i] = 0;
	}
}

void navigation(float* measData, float* powerCmd, float PPast[12][12], float* stateEst, float dt) {
	// recieves sensor (measured) data and outputs estimated states from Kalman Filter
	// helpful reference: http://robotsforroboticists.com/kalman-filtering/


	// measData: // {x, y, z1, z2, p1, q1, r1, ax1, ay1, az1, p2, q2, r2, ax2, ay2, az2}
	// cam: x, y, z1
	// sonar: z2
	// imu1: p1, q1, r1, ax1, ay1, az1
	// imu2: p2, q2, r2, ax2, ay2. az2


	bool takeMeasure = true;

	float accel[3] = { measData[AX1], measData[AY1], measData[AZ1] }; // we could include both weighted accelerometer measures

	// TODO: stateNew = A*statePast, to give x = A*statePast + B*action
	float A[12][12] = {};
	A[0][6] = cos(stateEst[PSI]);
	A[0][7] = -sin(stateEst[PSI]);
	A[1][6] = sin(stateEst[PSI]);
	A[1][7] = cos(stateEst[PSI]);
	A[2][8] = 1;
	A[9][9] = 1;
	A[4][10] = 1;
	A[5][11] = 1;
	A[6][4] = -g;
	A[7][3] = g;
	A[8][3] = -g;
	A[8][4] = -g;

	// float B[12][4] = {};

	float ATransp[12][12] = {};
	for (int i = 0; i < 12; ++i)
	{
		for (int j = 0; j < 12; ++j)
		{
			ATransp[j][i] = A[i][j];
		}
	}

	// C is the states where we're taking a measurement
	float C[3][12] = {};
	C[0][X] = 1;
	C[1][Y] = 1;
	C[2][Z] = 1;
	float CTransp[12][3] = {};
	CTransp[X][0] = 1;
	CTransp[Y][1] = 1;
	CTransp[Z][2] = 1;

	float pMat[12][12] = {};
	float Qc[12][12] = {};	// expected values of disturbance (w^2)
	float Rc[3][3] = {};	// expected values of measurement noise variance (v^2)
	Rc[X][X] = sigX * sigX;
	Rc[Y][Y] = sigY * sigY;
	Rc[Z][Z] = sigZ2 * sigZ2;

	float Kc[12][3] = {};	// how much we want to weight the error
	float I[12][12] = {};	// identity matrix

	float totalXEstDot[12] = {};
	float xEstDot1 = 0, xEstDot2 = 0, xEstDot3 = 0, xEstDot4 = 0;
	float xEst[12] = {};
	float xEstNew[12] = {};
	float stateMeas[12] = {};
	float stateEstPast[12] = {};

	for (int i = 0; i < 12; i++)
	{
		Qc[i][i] = 1;
		I[i][i] = 1;
		stateEstPast[i] = stateEst[i];
		xEst[i] = stateEst[i];
		for (int j = 0; j < 12; j++) {
			pMat[i][j] = PPast[i][j];
		}
	}

	stateMeas[X] = measData[X];
	stateMeas[Y] = measData[Y];
	stateMeas[Z] = measData[Z2];

	// Start the actual agorithm
	for (int i = 0; i < 12; i++)
	{
		// RK4 algorithm
		float k[4][12] = {}; // here, k is xEstDot

		stateDerivative(stateEstPast, k[0], accel, powerCmd);

		for (int i = 0; i < 12; i++)
			stateEst[i] = stateEstPast[i] + k[0][i] * dt / float(2);
		stateDerivative(stateEstPast, k[1], accel, powerCmd);

		for (int i = 0; i < 12; i++)
			stateEst[i] = stateEstPast[i] + k[1][i] * dt / float(2);
		stateDerivative(stateEstPast, k[2], accel, powerCmd);

		for (int i = 0; i < 12; i++)
			stateEst[i] = stateEstPast[i] + k[2][i] * dt;
		stateDerivative(stateEstPast, k[3], accel, powerCmd);

		for (int i = 0; i < 12; i++)
			totalXEstDot[i] = (k[0][i] + float(2) * k[1][i] + float(2) * k[2][i] + k[3][i]) / float(6);
		// but don't integrate yet until we add the measurements
	}

	// predictions
	float temp1[12][12] = {};
	float temp2[12][12] = {};
	float temp3[12][12] = {};

	float pMatTemp[12][12] = {};
	float pDot[12][12] = {};
	float pDot1[12][12] = {};
	float pDot2[12][12] = {};
	float pDot3[12][12] = {};
	float pDot4[12][12] = {};

	matMult(A, pMat, temp1); // [12x12 * 12x12 = 12x12]
	matMult(temp1, ATransp, temp2); // [12x12 * 12x12 = 12x12]
	matAdd(temp2, Qc, pDot1); 	// pDot1 = A*P*A' + Q [12x12 + 12x12 = 12x12]

	matScale(pDot1, dt / float(2), temp1);
	matAdd(pMat, temp1, temp2); // [12x12 + 12x12 = 12x12]
	matMult(A, temp2, temp1);
	matMult(temp1, ATransp, temp2);
	matAdd(temp2, Qc, pDot2); // pDot2 = A*(P + pDot1*dt/2)*A' + Q

	matScale(pDot2, dt / float(2), temp1);
	matAdd(pMat, temp1, temp2);
	matMult(A, temp2, temp1);
	matMult(temp1, ATransp, temp2);
	matAdd(temp2, Qc, pDot3); // pDot3 = A*(P + pDot2*dt/2)*A' + Q

	matScale(pDot3, dt, temp1);
	matAdd(pMat, temp1, temp2);
	matMult(A, temp2, temp1);
	matMult(temp1, ATransp, temp2);
	matAdd(temp2, Qc, pDot4); // pDot4 = A*(P + pDot3*dt)*A' + Q

	matScale(pDot2, 2, temp2);			// t2 = 2*pDot2
	matAdd(pDot1, temp2, temp1);		// t1 = pDot1 + 2*pDot2
	matScale(pDot3, 2, temp2);			// t2 = 2*pDot3 
	matAdd(temp1, temp2, temp3);		// t3 = pDot1 + 2*pDot2 + 2*pDot3
	matAdd(temp3, pDot4, temp1);		// t1 = pDot1 + 2*pDot2 + 2*pDot3 + pDot4
	matScale(temp1, float(1 / 6), pDot);// pDot = (1/6) * (pDot1 + 2*pDot2 + 2*pDot3 + pDot4)

	matScale(pDot, dt, temp1);
	matAdd(pMat, temp1, pMatTemp); // pMatTemp = pMat + pDot*dt. This is the first pMat before the measurements

	// my zoom failed me i think? 
	// Yeah, you're not there anymore...
	// I thought it was my whole wifi and i got scared this would die
	// It does that if your wifi is *too slow* and kicks you out of zoom. Discord? or just type lol
	// yea i tried relaunching it and it still didnt work. its beacuse the liveshare. Discord yup
	// we should just leave this in the code
	// why not? a little easter egg for our future selves

	// correct the Kalman Filter with measurements
	if (takeMeasure)
	{
		// make estimate of what the measures would be from xest = C*transp(Y)	
		float yEst[3] = {};
		float stateMeasEst[12] = { };
		//matMult2(C, xEst, yEst); // [3x12 * 12x1 = 3x1] 
		// Matrix multiplication of a 3x12 and 12x3 matrix = 3x3 matrix

		for (int i = 0; i < 3; i++)
		{
			for (int k = 0; k < 12; k++)
			{
				yEst[i] += C[i][k] * xEst[k];
			}
		}

		stateMeasEst[0] = yEst[0];
		stateMeasEst[1] = yEst[1];
		stateMeasEst[2] = yEst[2];

		// calculate Kc
		float temp6a[12][3] = {};
		float temp6b[3][12] = {};
		float temp6c[3][3] = {};
		float temp6d[3][3] = {};
		matMult5(pMatTemp, CTransp, temp6a);// ta = P * C'					[12x12 * 12x3 = 12x3]
		matMult0(C, pMatTemp, temp6b);		// tb = C * P					[3x12 * 12x12 = 3x12]
		matMult1(temp6b, CTransp, temp6c);	// tc = C * P * C'				[3x12 * 12x3 = 3x3]
		matAdd(temp6c, Rc, temp6d);			// td = C * P * C' + Rc			[3x3 * 3x3 = 3x3]
		matInv(temp6d, temp6c);				// tc = inv(C * pDot * C' + Rc)	[3x3]
		matMult3(temp6a, temp6c, Kc);		// Kc = P * C' * inv(C * P * C' + Rc)	[12x3 * 3x3 = 12x3]

		// Apply the measurement's contribution to xEstDot
		// this is the same as xhat = xhat + K*residual, where residual = C*(state - lastState)
		float temp12a[12] = {};
		float temp12b[12] = {};
		matMult4(Kc, C, temp1);						// t1 = Kc * C
		matSub(stateMeas, stateMeasEst, temp12a);	// ta = stateMeas - stateMeasEst			[12x1 - 12x1 = 12x1]
		matMult(temp1, temp12a, temp12b);			// Kc * C * (stateMeas - stateMeasEst)		[12x12 * 12x1 = 12x1]
		for (int i = 0; i < 12; i++) {
			temp12a[i] = totalXEstDot[i] + temp12b[i]; // newTotalXEstDot = totalXEstDot + Kc * C * (stateMeas - stateMeasEst)
		}

		// correct P estimate
		matMult4(Kc, C, temp1);			// t1 = Kc * C
		matScale(temp1, -1, temp2);		// t2 = -Kc*C
		matAdd(I, temp2, temp1);		// t1 = I + (-Kc*C)
		matMult(temp2, pMatTemp, PPast); // pMatNew = (I - Kc * C) * pMat
	}
	
	// actually finish integrating

	for (int i = 0; i < 12; i++) {
		xEstNew[i] = xEst[i] + totalXEstDot[i] * dt;
	}

	for (int i = 0; i < 12; i++) {
		stateEst[i] = xEstNew[i]; // this is just commented out so it doesn't break anything
	}
}

void guidance(float* stateEst, float* stateDes, float* xyzpsiErrPast, float* xyzpsiErrSum, float* stick) {
	float Ks = float(1);
	float Kp = float(1);
	float Ki = float(0.0002);
	float Kd = float(0);

	stateDes[PSI] = (stateDes[Y] - stateEst[Y]) / (stateDes[X] - stateEst[X]);

	float xyzpsiErr[4] = {};
	xyzpsiErr[0] = stateDes[X] - stateEst[X];
	xyzpsiErr[1] = stateDes[Y] - stateEst[Y];
	xyzpsiErr[2] = stateDes[Z] - stateEst[Z];
	xyzpsiErr[3] = stateDes[PSI] - stateEst[PSI];

	for (int i = 0; i < 4; i++) {
		xyzpsiErrSum[i] += xyzpsiErr[i];
	}

	stateDes[P] =   Kp*xyzpsiErr[1] + Ki*xyzpsiErrSum[1] + Kd*(xyzpsiErr[1] - xyzpsiErrPast[1]);
	stateDes[Q] = -(Kp*xyzpsiErr[0] + Ki*xyzpsiErrSum[0] + Kd*(xyzpsiErr[0] - xyzpsiErrPast[0]));
	stateDes[R] =   Kp*xyzpsiErr[3] + Ki*xyzpsiErrSum[3] + Kd*(xyzpsiErr[3] - xyzpsiErrPast[3]);
	stateDes[W] =   Kp*xyzpsiErr[2] + Ki*xyzpsiErrSum[2] + Kd*(xyzpsiErr[2] - xyzpsiErrPast[2]);

	stateDes[P] = 0; // temp
	stateDes[Q] = 0; // temp
	stateDes[R] = 0; // temp

	float pqrwErr[4] = {};
	pqrwErr[0] = stateDes[P] - stateEst[P];
	pqrwErr[1] = stateDes[Q] - stateEst[Q];
	pqrwErr[2] = stateDes[R] - stateEst[R];
	pqrwErr[3] = -(stateDes[W] - stateEst[W]);

	for (int i = 0; i < 4; i++) {
		stick[i] = Ks * pqrwErr[i];
		
		if (stick[i] > 1) stick[i] = 1;
		if (stick[i] < -1) stick[i] = -1;
		
	}

	for (int i = 0; i < 4; i++) {
		xyzpsiErrPast[i] = xyzpsiErr[i];
	}
}
/*

void readIMU(float* measData, MPU9250 mpu, LSM9DS1 lsm) {

	// --- From the AccelGyroMag.cpp example --- //
	// .\Examples\AccelGyroMag\AccelGyroMag.cpp

	// measData // {x, y, z1, z2, p1, q1, r1, ax1, ay1, az1, p2, q2, r2, ax2, ay2, az2}

	//if (check_apm()) { // not sure what this is or why it's in here, but it's in the example file
	//	return;
	//}


	float a1, a2, a3;
	float w1, w2, w3;

	// mpu
	mpu->update();
	mpu->read_accelerometer(&a1, &a2, &a3);
	mpu->read_gyroscope(&w1, &w2, &w3);
	measeData[AX1] = -a2;
	measeData[AY1] = -a1;
	measeData[AZ1] = a3;
	measeData[P1] = w2;
	measeData[Q1] = w1;
	measeData[R1] = -w3;

	// lsm
	lsm->update();
	lsm->read_accelerometer(&a1, &a2, &a3);
	lsm->read_gyroscope(&w1, &w2, &w3);
	measeData[AX2] = -a2;
	measeData[AY2] = -a1;
	measeData[AZ2] = a3;
	measeData[P2] = w2;
	measeData[Q2] = w1;
	measeData[R2] = -w3;
}
*/


void readSonar(float* stateMeas) {
	stateMeas[Z2] = 0;
}

void readCam(float* stateMeas) {
	stateMeas[X] = 0;
	stateMeas[Y] = 0;
	stateMeas[Z1] = 0;
}

void readRC(float* stick) {
	stick[ROLL] = 0;   // -1 to 1
	stick[PITCH] = 0;  // -1 to 1
	stick[YAW] = 0;    // -1 to 1
	stick[THRUST] = 0; // 0? to 1
}


// --- All the matrix math functions --- //

void matMult(float a[12][12], float* b, float* mul) {
	// Matrix multiplication of a 12x12 and 12x1 matrix
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			mul[i] += a[i][j] * b[j];
		}
	}
}
void matMult(float a[12][12], float b[12][12], float mul[12][12]) {
	// Matrix multiplication of two 12x12 matrices
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			for (int k = 0; k < 12; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matMult5(float a[12][12], float b[12][3], float mul[12][3]) {
	// Matrix multiplication of a 12x12 and 12x3 matrix = 12x3 matrix
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 12; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matMult4(float a[12][3], float b[3][12], float mul[12][12]) {
	// Matrix multiplication of a 12x3 and 3x12 matrix = 12x12 matrix
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matMult3(float a[12][3], float b[3][3], float mul[12][3]) {
	// Matrix multiplication of a 12x3 and 3x3 matrix = 12x3 matrix
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matMult2(float a[3][12], float* b, float* mul) {
	// Matrix multiplication of a 3x12 and 12x3 matrix = 3x3 matrix
	for (int i = 0; i < 3; i++)
	{
		for (int k = 0; k < 12; k++)
		{
			mul[i] += a[i][k] * b[k];
		}
	}
}
void matMult1(float a[3][12], float b[12][3], float mul[3][3]) {
	// Matrix multiplication of a 3x12 and 12x3 matrix = 3x3 matrix
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 12; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}
void matMult0(float a[3][12], float b[12][12], float mul[3][12]) {
	// Matrix multiplication of a 3x12 and 12x3 matrix = 3x3 matrix
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			for (int k = 0; k < 12; k++)
			{
				mul[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

void matScale(float a[12][12], float con, float mul[12][12]) {
	// Scale a 12x12 matrix by a constant
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			mul[i][j] = a[i][j] * con;
		}
	}
}
void matScale(float* a, float con, float* mul) {
	// Scale a 12x12 matrix by a constant
	for (int i = 0; i < 12; i++)
	{
		mul[i] = a[i] * con;
	}
}

void matAdd(float a[12][12], float b[12][12], float mul[12][12]) {
	// Matrix addition of two 12x12 matrices
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			mul[i][j] = a[i][j] + b[i][j];
		}
	}
}
void matAdd(float a[3][3], float b[3][3], float mul[3][3]) {
	// Matrix addition of two 3x3 matrices
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			mul[i][j] = a[i][j] + b[i][j];
		}
	}
}

void matSub(float* a, float* b, float* mul) {
	// Matrix subtraction of two 12x1 matrices
	for (int i = 0; i < 12; i++)
	{
		mul[i] = a[i] - b[i];
	}
}
void matInv(float a[3][3], float mul[3][3]) {
	// Matrix invserion of a 3x3 matrix
	float det = 0;
	float aT[3][3] = { };

	for (int i = 0; i < 3; i++)
	{
		det = det + (a[0][i] * (a[1][(i + 1) % 3] * a[2][(i + 2) % 3] - a[1][(i + 2) % 3] * a[2][(i + 1) % 3]));
	}

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			aT[j][i] = a[i][j];
		}
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			mul[i][j] = ((aT[(i + 1) % 3][(j + 1) % 3] * aT[(i + 2) % 3][(j + 2) % 3]) -
				(aT[(i + 1) % 3][(j + 2) % 3] * aT[(i + 2) % 3][(j + 1) % 3])) / det;
		}
	}

}
#endif // !"onboard.h"
