#include "stdafx.h"

#include "rMath/rMath.h"
using namespace rMath;

#include "rCommand/rCmdDefine.h"
#include "rxSDK/rxSDK.h"

#include "../controlsegway/controlsegwayCmd.h"

#include <fstream>
#include <iostream>
using namespace std;
#define MAX_SIZE 1000
char inputString[MAX_SIZE];
//#include "../controlXXXX/controlXXXXCmd.h"

#ifdef USING_NAMESPACE_RTERM
#include "rTerm/rTerm.h"
using namespace rTerm;
#endif

bool bContact = true;		// Enables/disables contact dynamics.
bool bQuit = false;			// Set this flag true to quit this program.
bool bRun = false;			// Set this flag true to activate the program.
// See OnKeyRun() function for the details.

#ifdef USING_NAMESPACE_RTERM
// Defining a macro, RPLAYER_AUTO_START, makes your program start rPlayer automatically.
// If you don't want rPlayer to be started automatically, comment out the code below.
#define RPLAYER_AUTO_START
// Defining a macro, RPLOT_AUTO_START, makes your program start rPlot automatically.
// If you don't want rPlot to be started automatically, comment out the code below.
#endif

void OnKeyQuit(void* data)
{
	PHYSICS_WORLD->deactivateWorld();
	bQuit = true;
}

void OnKeyRun(void* data)
{
	bRun = !bRun;
	if (bRun)
		PHYSICS_WORLD->activateWorld();
	else
		PHYSICS_WORLD->deactivateWorld();
}

void OnCommand(int key, void* data)
{
	rxControlInterface* control = (rxControlInterface*)data;
	switch (key)
	{
	case VK_W:
		control->command(MOVE_FORWARD);
		break;
	case VK_S:
		control->command(MOVE_BACKWARD);
		break;
	case VK_A:
		control->command(TURN_LEFT);
		break;
	case VK_D:
		control->command(TURN_RIGHT);
		break;
	case VK_SPACE:
		control->command(STOP);
		break;
	}
}


void SetKeyboardHandler(rxSystem* sys, rxControlInterface* control)
{
	PHYSICS_WORLD->addKeyboardEvent(VK_Q, OnKeyQuit);
	PHYSICS_WORLD->addKeyboardEvent(VK_TAB, OnKeyRun);

	printf("To run/pause simulation, press TAB key.\n");
	printf("To quit simulation, press 'q' key.\n");
	PHYSICS_WORLD->printLogMessage(_T("To run/pause simulation, press TAB key."));
	PHYSICS_WORLD->printLogMessage(_T("To quit simulation, press 'q' key."));

	printf(">>>Control Command<<<\n");
	printf("w : Go forward.\n");
	printf("s : Go backward.\n");
	printf("a : Go leftward.\n");
	printf("d : Go rightward.\n");

	PHYSICS_WORLD->printLogMessage(_T(">>>>Control Command<<<<"));
	PHYSICS_WORLD->printLogMessage(_T("w : Go forward"));
	PHYSICS_WORLD->printLogMessage(_T("s : Go backward"));
	PHYSICS_WORLD->printLogMessage(_T("a : Go leftward"));
	PHYSICS_WORLD->printLogMessage(_T("d : Go rightward"));

	PHYSICS_WORLD->addKeyboardEvent(VK_W, OnCommand, control);
	PHYSICS_WORLD->addKeyboardEvent(VK_S, OnCommand, control);
	PHYSICS_WORLD->addKeyboardEvent(VK_A, OnCommand, control);
	PHYSICS_WORLD->addKeyboardEvent(VK_D, OnCommand, control);
	PHYSICS_WORLD->addKeyboardEvent(VK_SPACE, OnCommand, control);

}

int _tmain(int argc, _TCHAR* argv[])
{


	const rTime delT = 0.005;						// simulation 시간간격 설정, Basic time increment.

	PHYSICS_WORLD->createWorld(bContact, delT);		// Create a world for simulation.
	if (!PHYSICS_WORLD->isWorldCreated())			// Check if a world is created successfully.
	{
		printf("Failed to create a physics world. Press any key to quit...");
		getchar();
		return -1;
	}

	PHYSICS_WORLD->setGravity(0, 0, -GRAV_ACC);		// Set gravitational vector.
	// It is only valid for simulation(virtual) world.
	// By default, it is already set as (0, 0, -GRAV_ACC).

	if (bContact)									// If contact dynamics is enabled, then create a infinite plane for the ground.
		PHYSICS_WORLD->createPlane(0, 0, 1, 0);

	printf("wait for connection..\n");
#ifdef RPLAYER_AUTO_START
	startPlayer("-ip 127.0.0.1 -port 5150");		// Start rPlayer.
	//startPlot("-ip 127.0.0.1 -port 5150");
#endif
	PHYSICS_WORLD->makeNetwork(1000);				// Create a network instance.
	// This function is blocked until a new client such as rPlayer is connected or timeout is occurred.

	string_type aml = _T("models/final/final_test.aml");
	//PHYSICS_WORLD->createEnvironment(_T("models/Environment/SimpleOutdoor/hmap1.eml"), _T("office"));
	//PHYSICS_WORLD->createEnvironment(_T("models/Environment/bk/up10.eml"), _T("office"));
	// Set AML file path to load including file extension, 'aml'.
	// It can be a relative path based on the program working directory or absolute file path.
	// Usually the working directory is set from the system environment variable, $(RLAB_BIN_PATH).
	string_type name = _T("test");					// Set the name of your model. Each name should be unique.

	HTransform T0;									// Initial position and orientation of your robot. 
	dVector q0;										// Initial joint position, q values of robot.
	rxSystem* sys = NULL;							// Loaded system.
	T0.r[2] = 0.30;									// 모델 초기시작 위치 설정, r[2]는 높이
	T0.r[0] = 0;
	T0.r[1] = 0;

	sys = PHYSICS_WORLD->createSystem(aml, name, T0, q0); // Load a system and put it into the world.
	if (!sys)										// Check if the system is created successfully.
	{
		printf("Failed to load a robot model(AML). Press any key to quit...");
		getchar();
		return -1;
	}

	PHYSICS_WORLD->initialize();					// Initialize the world.
	// Every system should be created before this function call.

	startPlot("-ip 127.0.0.1 -port 5150");

	rID plotidd = DATA_ACQUISITION->createPlot(_T("pitch"), eDataPlotType_TimeLine);
	rID plotiddd = DATA_ACQUISITION->createPlot(_T("wheel_vel"), eDataPlotType_TimeLine);
	rID plotidddd = DATA_ACQUISITION->createPlot(_T("sys"), eDataPlotType_TimeLine);
	rID plotiddddd = DATA_ACQUISITION->createPlot(_T("tilt_angle"), eDataPlotType_TimeLine);
	rID plotidddddd = DATA_ACQUISITION->createPlot(_T("torque"), eDataPlotType_TimeLine);
	rxDevice* gyro = sys->findDevice(_T("gyro")); //디바이스읽기
	rxDevice* tacho_lwheel = sys->findDevice(_T("tacho_lwheel"));
	rxDevice* tacho_rwheel = sys->findDevice(_T("tacho_rwheel"));
	rxDevice* enc_tilt = sys->findDevice(_T("enc_tilt"));
	rxDevice* tmotor = sys->findDevice(_T("tmotor"));
	//rxDevice* lmotor = sys->findDevice(_T("lmotor"));
	DATA_ACQUISITION->add(plotidd, gyro, eDeviceDataType_ReadFloat);
	DATA_ACQUISITION->add(plotiddd, tacho_lwheel, eDeviceDataType_ReadFloat);
	DATA_ACQUISITION->add(plotiddd, tacho_rwheel, eDeviceDataType_ReadFloat);
	DATA_ACQUISITION->add(plotidddd, sys, eSystemDataType_qdot);
	DATA_ACQUISITION->add(plotiddddd, enc_tilt, eDeviceDataType_ReadFloat);
	DATA_ACQUISITION->add(plotidddddd, tmotor, eDeviceDataType_ReadFloat);
	//DATA_ACQUISITION->add(plotidddddd, lmotor, eDeviceDataType_ReadFloat);

	rxControlInterface* control = NULL;
	if (sys)
	{
		int step = 1;								// Controller is updated once per 'step' simulation times.
		// When step is 0, the controller should be updated by user manually.
		control = PHYSICS_WORLD->createController(_T("controller_name"), sys, step);
		string_type control_plugin_path = _T("controls/controlsegway.dll");
		// Set your control plugin DLL file path relative to working directory.
		// Usually the working directory is set from the system environment variable, $(RLAB_BIN_PATH).
		control->setAlgorithmDll(control_plugin_path);
		control->setPeriod(step*delT);
		control->setNominalSystem(aml, name, T0, q0);
		control->initAlgorithm();
	}

	SetKeyboardHandler(sys, control);				// Setup keyboard handler.
	// See the function implementation of 'SetKeyboardHandler()' for the details.


	while (!bQuit)									// Program main-loop.
	{
		PHYSICS_WORLD->update();


	}
	DESTROY_DATA_ACQUISITION();
	DESTROY_PHYSICS_WORLD();						// Destroy all instances created.

#ifdef RPLAYER_AUTO_START
	stopPlayer();									// Exit all the rPlayer instances.
	//stopPlot();
#endif

	return 0;
}
