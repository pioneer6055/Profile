 /*
 *  Demo program for the Profile library
 *  requires heading value from a gyro and distance value from an encoder
 *  Author: Chester Marshall, mentor for 6055 and 5721
 */

#include "AHRS.h"
#include "PID.h"
#include "Profile.h"
#include "WPILib.h"

class Robot: public IterativeRobot
{
private:
	Joystick Stick;
	Victor MotorLeft,MotorRight;
	RobotDrive DriveTrain;
	Encoder TheEncoder;
	Profile AutoProfile;
	AHRS *Gyro;
	float HeadingOffset = 0.0f;
	double avgStickX = 0.0f;
	double avgStickY = 0.0f;

	// DistancePerPulse = (1/PulsesPerRevolution) * PI * WheelDiameter

	//for e4t encoder on 6055 bot
	//float InchesPerPulse = 0.05911; //must modify RobotInit and Profile.Set_Trapezoid if using inches
	//float FeetPerPulse = 0.0049;

	//for FIT0186 encoder on protobot
	float InchesPerPulse = 0.0184; //must modify RobotInit and Profile.Set_Trapezoid if using inches
	float FeetPerPulse = 0.00153;
	int AutoState = 0;
public:
	Robot() :
		Stick(0),
		MotorLeft(0),
		MotorRight(1),
		DriveTrain(MotorLeft,MotorRight),
		TheEncoder(0, 1, true, CounterBase:: k1X)  //DIO 0 and 1 channels
	   {
			//instantiate the navx
			try
			{
				Gyro = new AHRS(SPI::Port::kMXP,60);
			}
			catch (std::exception& ex )
			{
				std::string err_string = "Error instantiating navX MXP:  ";
				err_string += ex.what();
				DriverStation::ReportError(err_string.c_str());
			}
			if ( Gyro )
			{
				printf("NAVX Initialized OK\n");
				//LiveWindow::GetInstance()->AddSensor("AHRS", "Gyro", ahrs);
			}
	   }

	void RobotInit()
	{
		TheEncoder.SetDistancePerPulse(FeetPerPulse);
		DriveTrain.SetSafetyEnabled(false);
		MotorLeft.SetInverted(true);
		MotorRight.SetInverted(true);
	}

	void AutonomousInit()
	{
		//initialize the sequencer
		AutoState = 0;
		//set min/max speed range for forward/backward moves
		AutoProfile.ProfileMinSpeed = 0.5;
		AutoProfile.ProfileMaxSpeed = 1.0;
		//set min/max speed range for turning
		AutoProfile.ProfileMinTurnSpeed = 0.35;
		AutoProfile.ProfileMaxTurnSpeed = 0.75;
		//don't slow down between continuous movements
		AutoProfile.ProfileContinuous = true;
	}

	double GetHeading()
	{
		double offsetYaw = AutoProfile.GetNormalizedHeading(Gyro->GetYaw()) - HeadingOffset;
		if(offsetYaw < 0) offsetYaw += 360;
		return offsetYaw;
	}

	void ZeroHeading()
	{
		HeadingOffset = AutoProfile.GetNormalizedHeading(Gyro->GetYaw());
	}

	void ExecuteProfile()
	{
		AutoProfile.ExecuteProfile(GetHeading(),TheEncoder.GetDistance());
		DriveTrain.Drive(AutoProfile.OutputMagnitude,AutoProfile.Curve);
	}

	void AutonomousPeriodic()
	{
		switch(AutoState)  //autonomous sequencer
		{
			case 0:     //setup turn right and backup opposite
				ZeroHeading();
				TheEncoder.Reset();
				AutoProfile.Initialize();
				AutoProfile.ClearProfile();
				AutoProfile.AddMove(AutoProfile.kProfileForward,3);
				AutoProfile.AddTurn(90);
				AutoProfile.AddMove(AutoProfile.kProfileForward,3);
				AutoProfile.AddMove(AutoProfile.kProfileReverse,3);
				AutoProfile.AddTurn(180);
				AutoProfile.AddMove(AutoProfile.kProfileReverse,3);
				AutoProfile.ProfileLoaded = true;
				AutoState++;
				break;
			case 1:    //execute turn right and backup opposite
				if(AutoProfile.ProfileLoaded && !AutoProfile.ProfileCompleted) ExecuteProfile();
				else AutoState++;
				break;
			case 2:     //setup pause 3 seconds
				AutoProfile.Initialize();
				AutoProfile.ClearProfile();
				AutoProfile.AddPause(3000);
				AutoProfile.ProfileLoaded = true;
				AutoState++;
				break;
			case 3:    //execute pause 3 seconds
				if(AutoProfile.ProfileLoaded && !AutoProfile.ProfileCompleted) ExecuteProfile();
				else AutoState++;
				break;
			case 4:   //setup reverse to start
				AutoProfile.Initialize();
				AutoProfile.ClearProfile();
				AutoProfile.AddMove(AutoProfile.kProfileForward,3);
				AutoProfile.AddTurn(90);
				AutoProfile.AddMove(AutoProfile.kProfileForward,3);
				AutoProfile.AddMove(AutoProfile.kProfileReverse,3);
				AutoProfile.AddTurn(0);
				AutoProfile.AddMove(AutoProfile.kProfileReverse,3);
				AutoProfile.ProfileLoaded = true;
				AutoState++;
				break;
			case 5:    //execute reverse to start
				if(AutoProfile.ProfileLoaded && !AutoProfile.ProfileCompleted) ExecuteProfile();
				else AutoState++;
				break;
			default:
				DriveTrain.Drive(0.0,0.0);
				break;
		}
	}

	void TeleopInit() override
	{

	}

	void TeleopPeriodic() override
	{
		avgStickX = ffilter(Stick.GetRawAxis(0),avgStickX,0.25);
		avgStickY = ffilter(Stick.GetRawAxis(1),avgStickY,0.25);
		//drive with reversible tank steering via single joystick
		if (Stick.GetRawAxis(3) < 0)
			DriveTrain.ArcadeDrive(avgStickY,avgStickX*-1,false);
		else
			DriveTrain.ArcadeDrive(avgStickY*-1,avgStickX*-1,false);
	}

	double ffilter(double raw, double current, double lpf)
	{
		double weight = std::fmin(0.99999,std::fmax(lpf,0));
		return ((1-weight) * raw) + (weight * current);
	}
};

START_ROBOT_CLASS(Robot)
