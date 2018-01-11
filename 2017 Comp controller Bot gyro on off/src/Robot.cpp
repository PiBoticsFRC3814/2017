#include <Commands/custom_robot_control_command.h>
#include "WPILib.h"
#include "Commands/Command.h"
#include "CommandBase.h"
#include "CANTalon.h"
#include "ADIS16448_IMU.h"

class Robot: public IterativeRobot
{
	LiveWindow *lw;
	RobotDrive r; // basic robot object
	CANTalon LF;// Left-front motor
	CANTalon LR;// Left-right motor
	CANTalon RF;// Right-front motor
	CANTalon RR;// Right-rear motor
	CANTalon climb; // motor for climbing with the talon
	DigitalInput sw0; // this is a limit switch
	DigitalInput sw1; // this is also a limit switch
	DigitalInput sw2;
	Solenoid GearOpen; // this opens the gears
	Solenoid GearClose; // this closes the gears
	Solenoid USBSW;
	custom_drive_command cDrive; // custom mecanum class for driving
	XboxController rumble;
	ADIS16448_IMU *g;//ctrl click for more information on AIDS
	Relay l;// relay for controlling on/off of lights on camera
	Relay USBCAM;
	//NetworkTable *table; // this is not a physical table but rather a virtual one for passing variables between the Raspberry Pi (TM) and the RoboRIO
	CameraServer *camera; //this is a camera
	//CameraServer *camera1;
	Compressor *c = new Compressor(0); // this is a machine that compresses air for the solenoids to use
	///VISION VARIABLES
	double x = 0;
	double y = 0;
	double getX1;
	double getX2;
	double passZ;
	double screenmid = 320;
	bool itsees = false;
	/////////////////////
	bool runOnce = true;
	bool inPosition = false;
	bool driveWithGyro = true;
public:
	std::shared_ptr<NetworkTable> table;
	Robot():
		lw(NULL),
		r(LF, LR, RF, RR),
		LF(10),
		LR(11),
		RF(12),
		RR(13),
		climb(22),
		sw0(7), // 1
		sw1(8), // 2
		sw2(9), // 3
		GearOpen(2),
		GearClose(3),
		USBSW(0),
		cDrive(),
		rumble(2),
		g(),//gyro
		l(0,Relay::kForwardOnly),//light relay
		USBCAM(1,Relay::kForwardOnly),
		camera()//,
		//camera1()
		{
			r.SetExpiration(0.1);
			r.SetSafetyEnabled(false);
			table = NetworkTable::GetTable("datatable");
			table->SetServerMode();
			table->SetTeam(3816);
			table->Initialize();
		}

private:
	std::unique_ptr<Command> autonomousCommand;
	//SendableChooser *chooser;

	void RobotInit()
	{
		g = new ADIS16448_IMU; // declaration for AIDS dgyro


		camera->GetInstance()->StartAutomaticCapture(0);
		//camera1->GetInstance()->StartAutomaticCapture(1);

		camera->GetInstance()->SetSize(2);
		//camera1->GetInstance()->SetSize(2);

		lw = LiveWindow::GetInstance();
		//VISION
		getX1=0;
		getX2=0;
		passZ=0;
		table->PutNumber("Z", 0.0);
		table->PutNumber("X1", 0.0);
		table->PutNumber("X2", 0.0);
		l.Set(Relay::kOn);


	}
	void DisabledInit()
	{

	}
	void DisabledPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}
	void AutonomousInit()
	{
		l.Set(Relay::kOn);
		g->Reset();
		//Wait(1.0);

	}
	void AutonomousPeriodic()
	{
		Scheduler::GetInstance()->Run();
		cDrive.autonomous(autoChooser(),r,g,GearOpen,GearClose,l,x,y,getX1,getX2,passZ,screenmid,itsees, table,autonomousCommand, runOnce,inPosition,USBCAM);
	}
	void TeleopInit()
	{

		if (autonomousCommand != NULL)
			autonomousCommand->Cancel();
		climb.Set(0.0);
		table->SetPort(8080);
		l.Set(Relay::kOff);
		r.MecanumDrive_Cartesian(0.0,0.0,0.0,0.0);
	}
	void TeleopPeriodic()
	{
		Scheduler::GetInstance()->Run();
		c->SetClosedLoopControl(true);
		if (rumble.GetRawButton(9)){
			driveWithGyro = !driveWithGyro;
			Wait(0.25);
		}
		if (driveWithGyro)
		cDrive.drive_with_gyro(r,rumble,g,GearOpen,GearClose,USBSW,climb,l,USBCAM);
		if (!driveWithGyro)
		cDrive.drive_without_gyro(r,rumble,g,GearOpen,GearClose,USBSW,climb,l,USBCAM);
	}

	void TestPeriodic()
	{
		LiveWindow::GetInstance()->Run();
	}

	int autoChooser()
	{
		//this is matched to the switches on the board, all true/false logic in this function is inverted
		//****************************************************
		// sw0 sw1 sw2  returnS
		// T   T   T  	0
		// F   T   T    1
		// T   F   T    2
		// T   T   F    3
		// F   F   T    4
		// T   F   F    5
		// F   T   F    6
		// F   F   F    7
		//*****************************************************
		int a = 0;
			if (!sw0.Get())
			{

				if (!sw1.Get())
				{
					if (!sw2.Get())
					{
						a = 0;
					}
					else if (sw2.Get())
					{
						a = 3;
					}
				}
				else if (sw1.Get())
				{

					if (!sw2.Get())
					{
						a = 2;
					}
					else if (sw2.Get())
					{
						a= 5;
					}
				}
			}
			else if (sw0.Get())
			{
				if (!sw1.Get())
				{
					if (!sw2.Get())
					{
						a = 1;
					}
					else if (sw2.Get())
					{
						a = 6;
					}
				}
				else if (sw1.Get())
				{
					if (!sw2.Get())
					{
						a = 4;
					}
					else if (sw2.Get())
					{
						a = 7;
					}

				}
			}


		return a;
	}

};

START_ROBOT_CLASS(Robot)
