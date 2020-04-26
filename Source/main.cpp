#include <iostream>
#include <fstream>
#include <cstdio>
//#include <unistd>
#include <string>
#include <memory>
#include <time.h>
//#include "network.h"

#include "onboard.h"
/*
#include "Navio/Navio2/PWM.h"
#include "Navio/Navio2/ADC_Navio2.h"
#include "Navio/Navio2/Led_Navio2.h"
#include "Navio/Navio2/LSM9DS1.h"
#include "Navio/Navio2/RCInput_Navio2.h"
#include "Navio/Navio2/RCOutput_Navio2.h"
#include "Navio/Navio2/RGBled.h"
#include "Navio/Common/ADC.h"
#include "Navio/Common/gpio.h"
#include "Navio/Common/I2Cdev.h"
#include "Navio/Common/InertialSensor.h"
#include "Navio/Common/Led.h"
#include "Navio/Common/MPU9250.h"
#include "Navio/Common/MS5611.h"
#include "Navio/Common/RCInput.h"
#include "Navio/Common/RCOutput.h"
#include "Navio/Common/SPIdev.h"
#include "Navio/Common/Ublox.h"
#include "Navio/Common/Util.h"
using namespace Navio;
*/
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

	// autonomous or RC? this will aquired from ground terminal command 
	bool autonomyEnabled = true;

	// simulation or live?
	bool simulationEnabled = true;

	// bypass estimation and pass true state straight to guidance?
	bool trueStateOnlyEnabled = true;

	// set time properties
	int step = 0;
	float t = (float)0;
	const float dt = (float)0.001;
	const float endt = (float)15;
	
	// initialize csv writing (SIMULATION only)
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

	// --test area--
	
	//while (t <= endt - dt) {
	//	step++;
	//	t += dt;
	//	vehicleModel(stateTrue, stateTrueDot, aTrue, powerCmd, dt);
	//}

	//cout << "test completed";
	//return 0;
	// -------------


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
			readIMU(measData);
			readSonar(measData);
			readCam(measData);
		}

		navigation(measData, powerCmd, PPast, stateEst);
		
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


	cout << "completed";
	return 0;
}