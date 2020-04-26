#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <memory>
#include <time.h>

#include "onboard.h"
#include "offboard.h"

using namespace std;

int main() {
	// initilization
	float xyzpsiErrPast[4] = {};
	float  xyzpsiErrSum[4] = {};
	float     stickAuto[4] = {};
	float       stickRC[4] = {};
	float         stick[4] = {};
	float      powerCmd[4] = {};
	float    stateTrue[12] = {};
	float stateTrueDot[12] = {};
	float     stateEst[12] = {};
	float     stateDes[12] = {};
	float     measData[16] = {}; // {x, y, z1, z2, p1, q1, r1, ax1, ay1, az1, p2, q2, r2, ax2, ay2, az2}
	float    PPast[12][12] = {};
	float         aTrue[3] = {};
	
	stateTrue[X] = -6; // starts 6 meters from landing pad in the x direction
	stateTrue[Y] =  0; // starts in line in the y direction
	stateTrue[Z] = -4; // starts 4 meters above the ground
	
	stateDes[X] =  0; // wants to reach landing pad which is at 0,0
	stateDes[Y] =  0; // wants to reach landing pad which is at 0,0
	stateDes[Z] = -4; // wants to stay 4 meters above the ground

	stateEst[X] = -6; // --
	stateEst[Y] =  0; // init kalman filter
	stateEst[Z] = -4; // --

	// autonomous or RC? this will be aquired from ground terminal command 
	bool autonomyEnabled = true;

	// simulation or live?
	bool simulationEnabled = true;

	// bypass estimation and pass true state straight to guidance?
	bool trueStateOnlyEnabled = true;


	// initalize sensors
	auto mpu = std::unique_ptr <InertialSensor>{ new MPU9250() };
	auto lsm = std::unique_ptr <InertialSensor>{ new LSM9DS1() };

	mpu->initialize();
	lsm->initialize();


	// initialize csv writing (SIMULATION only)
	/*
	ofstream myfile("dataLog.csv");

	string labelTime = "t,";
	string labelStick = "stick_ROLL,stick_PITCH,stick_YAW,stick_THRUST,";
	string labelPowerCmd = "power_1,power_2,power_3,power_4,";
	string labelTrue = "x_TRUE,y_TRUE,z_TRUE,phi_TRUE,the_TRUE,psi_TRUE,u_TRUE,v_TRUE,w_TRUE,p_TRUE,q_TRUE,r_TRUE,";
	string labelTrueDot = "x_DOT_TRUE,y_DOT_TRUE,z_DOT_TRUE,phi_DOT_TRUE,the_DOT_TRUE,psi_DOT_TRUE,u_DOT_TRUE,v_DOT_TRUE,w_DOT_TRUE,p_DOT_TRUE,q_DOT_TRUE,r_DOT_TRUE,";
	string labelMeas = "x_MEAS,y_MEAS,z1_MEAS,z2_MEAS,p1_MEAS,q1_MEAS,r1_MEAS,ax1_MEAS,ay1_MEAS,az1_MEAS,p2_MEAS,q2_MEAS,r2_MEAS,ax2_MEAS,ay2_MEAS,az2_MEAS,";
	string labelEst = "x_EST,y_EST,z_EST,phi_EST,the_EST,psi_EST,u_EST,v_EST,w_EST,p_EST,q_EST,r_EST,";
	string labelDes = "x_DES,y_DES,z_DES,phi_DES,the_DES,psi_DES,u_DES,v_DES,w_DES,p_DES,q_DES,r_DES,";

	myfile << labelTime << labelStick << labelPowerCmd << labelTrue << labelTrueDot << labelMeas << labelEst << labelDes << "\n";
	*/
	// initilize time/rate properties
	int step = 0;
	float t = (float)0;
	const float dt = (float)0.001; // rate = 1/.001 = 1000Hz
	const float endt = (float)15;

	int downSampleNav = 50; // 1000Hz / 50 = 20Hz
	float dtNav = dt * downSampleNav;

	float tOld = 0;
	

	// -- first linux/imu test --

	while (t <= endt - dt) {
		t += dt;

		float a1, a2, a3;
		float w1, w2, w3;

		// mpu
		mpu->update();

		mpu->read_gyroscope(&w1, &w2, &w3);
		mpu->read_accelerometer(&a1, &a2, &a3);
		measData[P1] = w2;
		measData[Q1] = w1;
		measData[R1] = -w3;
		measData[AX1] = -a2;
		measData[AY1] = -a1;
		measData[AZ1] = a3;
		

		// lsm
		lsm->update();
		lsm->read_gyroscope(&w1, &w2, &w3);
		lsm->read_accelerometer(&a1, &a2, &a3);
		measData[P2] = w2;
		measData[Q2] = w1;
		measData[R2] = -w3;
		measData[AX2] = -a2;
		measData[AY2] = -a1;
		measData[AZ2] = a3;
		

		for (int i = 0; i < 12; i++) {
			cout << measData[P1 + i] << ", ";
		}
		cout << "\n";

	}
	

	// ----------- end -----------

	/* 
	// loopdy loop
	while (t <= endt - dt) {
		step++;
		t += dt;
		
		if (t > 5)
			stateDes[Z] = -6;

		if (simulationEnabled) {
			vehicleModel(stateTrue, stateTrueDot, aTrue, powerCmd, dt);
			sensorModel(stateTrue, aTrue, measData);
		}
		else {
			readIMU(measData, mpu, lsm);
			readSonar(measData);
			readCam(measData);
		}

		if (step % downSampleNav == 0) { // 1000Hz / 50 = 20Hz
			navigation(measData, powerCmd, PPast, stateEst, dtNav); // dt will need to be the sample time when we run this at a different rate
		}

		if (trueStateOnlyEnabled) {
			guidance(stateTrue, stateDes, xyzpsiErrPast, xyzpsiErrSum, stickAuto);
		}
		else {
			guidance(stateEst, stateDes, xyzpsiErrPast, xyzpsiErrSum, stickAuto);
		}

		readRC(stickRC);

		if (autonomyEnabled) {
			for (int i = 0; i < 4; i++) {
				stick[i] = stickAuto[i];
			}
		}
		else {
			for (int i = 0; i < 4; i++) {
				stick[i] = stickRC[i];
			}
		}
		vehicleController(stick, measData, powerCmd);
		
		myfile << t << ",";
		for (int i = 0; i < 4; i++)  { myfile << stick[i]        << ","; }
		for (int i = 0; i < 4; i++)  { myfile << powerCmd[i]     << ","; }
		for (int i = 0; i < 12; i++) { myfile << stateTrue[i]    << ","; }
		for (int i = 0; i < 12; i++) { myfile << stateTrueDot[i] << ","; }
		for (int i = 0; i < 16; i++) { myfile << measData[i]     << ","; }
		for (int i = 0; i < 12; i++) { myfile << stateEst[i]     << ","; }
		for (int i = 0; i < 12; i++) { myfile << stateDes[i]     << ","; }

		myfile << "\n";
	}
	
	myfile.close();
	*/


	cout << "completed";
	return 0;
}