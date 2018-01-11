/*
 * MechDrive.h
 *
 *  Created on: May 2, 2016
 *      Author: PiBotics
 */
#ifndef SRC_COMMANDS_CUSTOM_ROBOT_CONTROL_COMMAND_H_
#define SRC_COMMANDS_CUSTOM_ROBOT_CONTROL_COMMAND_H_

#include "WPILib.h"
#include "CANTalon.h"
class custom_drive_command: public IterativeRobot
{


	public:
		void default_constructor();
		void drive_without_gyro(RobotDrive&r,XboxController&rumble,ADIS16448_IMU *g,Solenoid&GearOpen,Solenoid&GearClose,Solenoid&USBSW,CANTalon&climb,Relay&l,Relay&USBCAM);
		void drive_with_gyro(RobotDrive&r,XboxController&rumble,ADIS16448_IMU *g,Solenoid&GearOpen,Solenoid&GearClose,Solenoid&USBSW,CANTalon&climb,Relay&l,Relay&USBCAM);
		void autonomous(int autoChoicae,RobotDrive&r,ADIS16448_IMU *g,Solenoid&GearOpen,Solenoid&GearClose,Relay&l, double&x,double&y,double&getX1,double&getX2,double&passZ,double&screenmid,bool&itsees,std::shared_ptr<NetworkTable>&table,std::unique_ptr<Command>&autonomousCommand,bool&runOnce,bool&inPosition,Relay&USBCAM);

};


#endif
