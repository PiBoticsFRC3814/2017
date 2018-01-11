/*
 * InvictusAutomaton.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: PiBotics
 */
#include <Commands/custom_robot_control_command.h>
#include "WPILib.h"
#include "ADIS16448_IMU.h"
	bool PIsOn = false; //checks if pneumatics are opened or closed
	bool LIsOn = true;
	//bool shouldIStayOrShouldIGo = false;
	bool camOn = true;
	int count7 = 0;
	bool gyroOnce = true;
void custom_drive_command::default_constructor()
{

}
//Default Constructor, does not use gyro
void custom_drive_command::drive_without_gyro(RobotDrive&r,XboxController&rumble,ADIS16448_IMU *g,Solenoid&GearOpen,Solenoid&GearClose,Solenoid&USBSW,CANTalon&climb,Relay&l,Relay&USBCAM)
{
		frc::Scheduler::GetInstance()->Run();
		double x, y, z = 0.0;

		z = rumble.GetRawAxis(2);
		y = rumble.GetRawAxis(1);
		x = rumble.GetRawAxis(0);
		r.MecanumDrive_Cartesian(x, y, z, -g->GetAngleZ());
	//releases gear
	if (rumble.GetRawButton(4))
	{
		if (PIsOn)
		{
			GearOpen.Set(true);
			Wait(0.25);//was .5
			GearOpen.Set(false);
			PIsOn = false;
		}
		else if (!PIsOn)
		{
			GearClose.Set(true);
			Wait(0.25);//was .5
			GearClose.Set(false);
			PIsOn = true;
		}
	}
	if (rumble.GetRawButton(1))
		{
		USBSW.Set(true);
		if (camOn)
		{
			USBCAM.Set(Relay::kOn);
			Wait(0.1);
			camOn = false;
		}
		else
		{
			USBCAM.Set(Relay::kOff);
			Wait(0.1);
			camOn = true;
		}
		}
	else
		{
		USBSW.Set(false);
		}
	//climb

		if (rumble.GetRawButton(8))
		{
			climb.Set(-1.0);//-1 for comp bot
		}
		else
		{
			climb.Set(0.0);
		}

}
//Default TeleOP
void custom_drive_command::drive_with_gyro(RobotDrive&r,XboxController&rumble,ADIS16448_IMU *g,Solenoid&GearOpen,Solenoid&GearClose,Solenoid&USBSW,CANTalon&climb,Relay&l,Relay&USBCAM)
{
//begin Mecanum drive code


	frc::Scheduler::GetInstance()->Run();
			double x, y, z = 0.0;

			z = rumble.GetRawAxis(2);
			y = rumble.GetRawAxis(1);
			x = rumble.GetRawAxis(0);
			r.MecanumDrive_Cartesian(x, y, z, -g->GetAngleZ());

	//r.MecanumDrive_Cartesian(xin*.50, yin*.50, zin*.50, -g->GetAngleZ());

	//end Mecanum drive code

	//releases gear
	if (rumble.GetRawButton(4))
	{
		if (PIsOn)
		{
			GearOpen.Set(true);
			Wait(0.25);//was .5
			GearOpen.Set(false);
			PIsOn = false;
		}
		else if (!PIsOn)
		{
			GearClose.Set(true);
			Wait(0.25);//was .5
			GearClose.Set(false);
			PIsOn = true;
		}
	}

	//resets gyro
	if (rumble.GetRawButton(10))
		{
			g->Reset();
		}
	if (rumble.GetRawButton(1))
		{
		USBSW.Set(true);
		if (camOn)
		{
			USBCAM.Set(Relay::kOn);
			Wait(0.1);
			camOn = false;
		}
		else
		{
			USBCAM.Set(Relay::kOff);
			Wait(0.1);
			camOn = true;
		}
		}
	else
		{
		USBSW.Set(false);
		}

	//climb

	if (rumble.GetRawButton(8))
	{
		climb.Set(-1.0);//-1 for comp bot
	}
	else
	{
		climb.Set(0.0);
	}
	//slow down the robot
	/*if (j1.GetRawButton(6))
	{
		shouldIStayOrShouldIGo = !shouldIStayOrShouldIGo;
		Wait(0.5);
	}*/
	//camera
	//if (j2.GetRawButton(6)==true)
		//camservo.Set(1.0);
//	if (	.GetRawButton(7)==true)
	//	g->Reset();
}
// Default Autonomous
void custom_drive_command::autonomous(int autoChoice,RobotDrive&r,ADIS16448_IMU *g,Solenoid&GearOpen,Solenoid&GearClose,Relay&l, double&x,double&y,double&getX1,double&getX2,double&passZ,double&screenmid,bool&itsees,std::shared_ptr<NetworkTable>&table,std::unique_ptr<Command>&autonomousCommand,bool&runOnce,bool&inPosition,Relay&USBCAM)
{	//****************************************************
	// sw0 sw1 sw2  returns  runs
	// T   T   T  	0		 straight across baseline  // no vision
	// F   T   T    1		 straight across baseline and back, right side // no vision
	// T   F   T    2		 straight across baseline and back, left side // no vision
	// T   T   F    3		 places peg straight across w/vision
	// F   F   T    4		 places right side peg w/vision
	// T   F   F    5		 places left side peg w/vision
	// F   T   F    6
	// F   F   F    7		 places peg straight across // no vision
	//*****************************************************
	//int count = 0;
	int pixelDead = 3;
	if (autoChoice == 0 && runOnce)
	{
		r.MecanumDrive_Cartesian(0.0,-0.5,0.0,0.0);
		Wait(1.0);
		r.MecanumDrive_Cartesian(0.0,0.0,0.0,0.0);
		runOnce = false;
	}
	if (autoChoice == 1 && runOnce)
	{
		r.MecanumDrive_Cartesian(0.0,-0.5,0.0,0.0);
		Wait(2.0);
		r.MecanumDrive_Cartesian(0.0,0.0,0.0,0.0);
		r.MecanumDrive_Cartesian(0.0,0.5,0.0,0.0);
		Wait(1.5);
		r.MecanumDrive_Cartesian(0.0,0.0,0.0,0.0);
		runOnce = false;
	}
	if (autoChoice == 2&& runOnce)
	{
		r.MecanumDrive_Cartesian(0.0,-0.5,0.0,0.0);
		Wait(2.0);
		r.MecanumDrive_Cartesian(0.0,0.0,0.0,0.0);
		r.MecanumDrive_Cartesian(0.0,0.5,0.0,0.0);
		Wait(1.5);
		r.MecanumDrive_Cartesian(0.0,0.0,0.0,0.0);
		runOnce = false;
	}
	if (autoChoice == 3)
		{
		double rotate;
		getX1 = table->GetNumber("X1",0.0);
		getX2 = table->GetNumber("X2",0.0);
		passZ = (getX1 + getX2)/2;
		table->PutNumber("G",g->GetAngleZ());


		if (passZ < 0)
			passZ*=-1;
				if (getX1!=getX2)
				{
					table->PutNumber("Z",passZ);
					itsees = true;
				}
				else
					itsees = false;
				if ( g->GetAngleZ()> 0.1)
					rotate = 0.05;
				else if (g->GetAngleZ()< -0.1)
					rotate = -0.05;
				else
					rotate = 0.0;
				if (itsees)
				{
				//r.MecanumDrive_Cartesian(0.0, -0.2, 0.0, 0.0);
					if ((passZ) < screenmid-pixelDead)
					{
						//r.MecanumDrive_Cartesian(-0.2, -0.2,rotate, 0.0);
						r.MecanumDrive_Cartesian(0.2, -0.2,rotate, 0.0);
					}
					else if ((passZ) > screenmid+pixelDead)
					{
						//r.MecanumDrive_Cartesian(0.2, -0.2, rotate, 0.0);
						r.MecanumDrive_Cartesian(-0.2, -0.2, rotate, 0.0);
					}
					else
						r.MecanumDrive_Cartesian(0.0, -0.2, 0.0, 0.0);
					}
					else
					{
						//r.MecanumDrive_Cartesian(0.0, -0.2, 0.0, 0.0);
						r.MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					}
		}
	//side peg
	//seems to work; need to test on a field.
	if (autoChoice == 4) //THis should be the master. copy this and make necessary negatives after it is finalized.
		{
				double rotate;
				getX1 = table->GetNumber("X1",0.0);
				getX2 = table->GetNumber("X2",0.0);
				passZ = (getX1 + getX2)/2;
				table->PutNumber("G",g->GetAngleZ());
				if (runOnce)
				{
					r.MecanumDrive_Cartesian(0.0, -0.5, 0.0, 0.0);
					Wait(1.15);
					r.MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
					runOnce = false;
				}
				if (g->GetAngleZ() <= 30)
				{
					r.MecanumDrive_Cartesian(0.0, 0.0, -0.5, 0.0); //rotates left, positive angle

				}
				else
				{
					inPosition = true;
				}
			if (inPosition)
			{//g->Reset(); //may or may not need this, resets gyro for easier skew correction
				if (passZ < 0)
					passZ*=-1;  // passZ = passZ * -1;
						if (getX1!=getX2)
						{
							table->PutNumber("Z",passZ);
							itsees = true;
						}
						else
							itsees = false;
						if ( g->GetAngleZ()> 60.1)
							rotate = 0.1;
						else if (g->GetAngleZ()< 59.9)
							rotate = -0.1;
						else
							rotate = 0.0;
						if (itsees)
						{
						//r.MecanumDrive_Cartesian(0.0, -0.2, 0.0, 0.0);
							if ((passZ) < (screenmid-20)+pixelDead)
							{
								//r.MecanumDrive_Cartesian(-0.2, -0.2,rotate, 0.0);
								r.MecanumDrive_Cartesian(0.2, -0.2,rotate, 0.0);
							}
								else if ((passZ) > (screenmid-20)-pixelDead)
							{
								//r.MecanumDrive_Cartesian(0.2, -0.2, rotate, 0.0);
								r.MecanumDrive_Cartesian(-0.2, -0.2, rotate, 0.0);
							}
							else
								r.MecanumDrive_Cartesian(0.0, -0.20, 0.0, 0.0);
							}
						else
						{
							//r.MecanumDrive_Cartesian(0.0, -0.2, 0.0, 0.0);
							r.MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
						}
			}
		}
	if (autoChoice == 5)
		{
		double rotate;
		getX1 = table->GetNumber("X1",0.0);
		getX2 = table->GetNumber("X2",0.0);
		passZ = (getX1 + getX2)/2;
		table->PutNumber("G",g->GetAngleZ());
			//
						if (runOnce)
						{
							r.MecanumDrive_Cartesian(0.0, -0.5, 0.0, 0.0);
							Wait(1.25);
							r.MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
							runOnce = false;
						}

						if (g->GetAngleZ() >= -30 && !inPosition)
						{
							r.MecanumDrive_Cartesian(0.0, 0.0, 0.5, 0.0); // rotate right, negative angle

						}
						else
						{
							inPosition = true;
							r.MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
						}
					if (inPosition)
					{
								if (getX1!=getX2)
								{
									table->PutNumber("Z",passZ);
									itsees = true;
								}
								else
									itsees = false;
								if ( g->GetAngleZ()> -59.9)
									rotate = 0.1;
								else if (g->GetAngleZ()< -60.1)
									rotate = -0.1;
								else
									rotate = 0.0;
								if (itsees)
								{

									if ((passZ) < screenmid-pixelDead)
									{

										r.MecanumDrive_Cartesian(0.2, -0.2,rotate, 0.0);
									}
									else if ((passZ) > screenmid+pixelDead)
									{

										r.MecanumDrive_Cartesian(-0.2, -0.2, rotate, 0.0);
									}
									else
										r.MecanumDrive_Cartesian(0.0, -0.2, rotate, 0.0);
								}
								else
								{

									r.MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
								}
					}
		}

	if (autoChoice == 6)
		{


		}
	if (autoChoice == 7)// && runOnce)
		{
		//added

		double rotate;
		if(count7<210)//250 was ~6 sec
		{
			if ( g->GetAngleZ()> 0.1)
				rotate = 0.1;
			else if (g->GetAngleZ()< -0.1)
				rotate = -0.1;
			else
				rotate = 0.0;
			//end added
				r.MecanumDrive_Cartesian(0.0,-0.35,rotate,0.0);//r.MecanumDrive_Cartesian(0.0,-0.3,0.0,0.0);
				//Wait(4.5);//3.5 @ fri morning
				//runOnce = false;
				Wait(0.01);
				count7++;

		}
		else
		{
						GearOpen.Set(true);
						Wait(0.25);//was .5
						GearOpen.Set(false);
						//PIsOn = false;
		}
		r.MecanumDrive_Cartesian(0.0,0.0,0.0,0.0);
		}



}

