/*
 * Profile.cpp
 *
 *  Created on: Oct 13, 2016
 *      Author: Chester Marshall, mentor for 6055 and 5721
 *
 *  This library can store a set of movement commands and execute the commands
 *  to drive the robot in a pre-set pattern.  This is most useful for autonomous mode.
 *  Right now four commands are implemented:
 *     MOVE  (drive in a straight line for a certain distance)
 *     TURN  (turn to a new heading)
 *     PAUSE (pause for a period of milliseconds)
 *     CURVE (drive in a curved line for a certain distance)
 *
 *	02/02/2017   -  CRM  -  corrected GetNormalizedError function (this early version used in competition for 2017)
 *	02/24/2017   -  CRM  -  added pause command
 *	02/29/2017   -  CRM  -  added trapezoid move profile (discrete version using distance slices)
 *	04/14/2017   -  CRM  -  added ProfileContinuous option - don't slow to zero between consecutive commands
 *	04/17/2017   -  CRM  -  removed steer command and added curve
 *	04/28/2017   -  CRM  -  changed static arrays to vector of struct
 *	11/16/2017   -  CRM  -  fixed bug in initializing struct values, cleaned up printf's
 *	11/21/2017   -  CRM  -  simplified function parameters, using feet unit for distance by default
 *
 */

#ifndef Profile_h
#define Profile_h

#include "PID.h"
#include "stdlib.h"
#include "Timer.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

struct ProfileParams
{
	double Command = 0.0f;
    double MinSpeed = 0.0f;
    double MaxSpeed = 0.0f;
    double TgtDistance = 0.0f;
    double TgtHeading = 0.0f;
    double PauseTime = 0.0f;
    double Curve = 0.0f;
    bool StartFlag = false;
    bool DoneFlag = false;
};

class Profile
{
private:
	std::vector <ProfileParams> Steps;
	uint StepNDX;
	double StartDistance;
	double MoveStartHeading;
	uint64_t MoveStartTime = 0;
	uint64_t PauseTime = 0;
	uint64_t ElapsedTime = 0;
	double MoveSteps = 128;
	double MoveMinSpeed;
	double MoveMaxSpeed;
	double MoveNextSpeed;
	double MoveLastSpeed;
	double MoveAccel;
	double MoveFirstCorner;
	double MoveSecondCorner;
	double MoveSlice;
	double MoveTarget;
	double MoveLastDist = 0.0f;
	uint MoveDistCount = 0;
	PID TurnPID;
	PID SteerPID;

public:
	typedef enum {kProfileForward,kProfileReverse} DirectionType;

	bool ProfileLoaded = false;
	bool ProfileContinuous = false;
	bool ProfileCompleted = false;
	int  ProfileStep = 0;
	double ProfileMinSpeed = 0.35;
	double ProfileMaxSpeed = 1.00;
	double ProfileMinTurnSpeed = 0.35;
	double ProfileMaxTurnSpeed = 0.75;
	double ProfileSteerKp = 0.01;
	double ProfileSteerKi = 0.00;
	double ProfileSteerKd = 0.00;
	double ProfileTurnKp = 0.05;
	double ProfileTurnKi = 0.00;
	double ProfileTurnKd = 0.00;
	float OutputMagnitude;
	float Curve;



    Profile();
    //call this before doing anything else
    void Initialize();
    //call this to zero profile steps array
    int ClearProfile();
    //call this to add move step to profile array
    int AddMove(DirectionType Direction, double TgtDistance);
    //call this to add turn step to profile array
    int AddTurn(double TgtHeading);
    //call this to add pause step to profile array
    int AddPause(double mSecs);
    //call this to add curve step to profile array
    int AddCurve(DirectionType Direction, double TgtDistance, double Curve);
    //call this repeatedly in AutonomousPeriodic
    //then set .Drive method with Profile.OutputMagnitude,Profile.Curve
    void ExecuteProfile(double heading, double distance);

    //********* INTERNAL METHODS **********
    //normalize heading value to 0-360 degrees
    double GetNormalizedHeading(double heading);
    //compute change in heading
    double GetNormalizedError(double heading, double newHeading);
    //enforce limits of -1 to 1
	double Clamp(double steerRate);
	//setup motion profile
	void Set_Trapezoid(double tgtValue, double minSpeed, double maxSpeed, double lastSpeed, double nextSpeed);
	//call repeatedly to execute motion profile based on distance feedback
	double Get_Trapezoid(double curDist);
};

#endif
