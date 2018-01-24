/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include "WpiLib.h"
#include "Commands/ExampleCommand.h"
#include "Commands/MyAutoCommand.h"
#include "ctre/Phoenix.h"
#include "Commands/ExampleCommand.h"
#include "Commands/MyAutoCommand.h"

class Robot : public frc::IterativeRobot {
	WPI_TalonSRX *lf = new WPI_TalonSRX(4);
	WPI_TalonSRX *lr = new WPI_TalonSRX(3);
	WPI_TalonSRX *rf = new WPI_TalonSRX(1);
	WPI_TalonSRX *rr = new WPI_TalonSRX(5);
	WPI_TalonSRX *shoot = new WPI_TalonSRX(2);
	MecanumDrive *myRobot = new MecanumDrive(*lf,*lr,*rf,*rr);
	Joystick *stick = new Joystick(0);
	Solenoid *fire = new Solenoid(0);
	Solenoid *reload = new Solenoid(1);
	Compressor *comp = new Compressor(0);
	double xin,yin,zin = 0.0;
	bool armed = false;


	public:
	/*Robot():
		lf(0),
		lr(1),
		rf(2),
		rr(3)
	{

	}*/
	void RobotInit() override {
		m_chooser.AddDefault("Default Auto", &m_defaultAuto);
		m_chooser.AddObject("My Auto", &m_myAuto);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		comp->SetClosedLoopControl(true);
		//rf->SetInverted(true);
		//rr->SetInverted(true);
		//myRobot->SetDeadband(0.2);
	}

	/**
	 * This function is called once each time the robot enters Disabled
	 * mode.
	 * You can use it to reset any subsystem information you want to clear
	 * when
	 * the robot is disabled.
	 */
	void DisabledInit() override {}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to
	 * the
	 * chooser code above (like the commented example) or additional
	 * comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		std::string autoSelected = frc::SmartDashboard::GetString(
				"Auto Selector", "Default");
		if (autoSelected == "My Auto") {
			m_autonomousCommand = &m_myAuto;
		} else {
			m_autonomousCommand = &m_defaultAuto;
		}

		m_autonomousCommand = m_chooser.GetSelected();

		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Start();
		}
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Cancel();
			m_autonomousCommand = nullptr;
		}

	}

	void TeleopPeriodic() override
	{
		double deadzone = .1;


		//below is custom deadzSone code.  we check that the joystick values are greater than the deadzone value.
		//if it is greater than the deadzone then it will pass the value to a variable
		//we then reinitialize the variable to a zero value so the speed isn't sudden

		if (stick->GetX()> deadzone || stick->GetX()< -deadzone){  //x axis deadzone check
				xin = stick->GetX();

				if (xin<0.0){
					xin = xin +0.1;
				} //re initialize positive x values

				else if (xin > 0.0) {
					xin = xin - 0.1;
				} //re initialize negative x values
			}
			else xin = 0; // if we are lower than the deadzone set the value to zero -- this avoids "locking in" a non-zero value


			if (stick->GetY()> deadzone || stick->GetY()< -deadzone){ //y axis deadzone check
				yin = (stick->GetY());

				if (yin<0.0){
					yin = yin +0.1;
				}//re initialize positive y values

				else if (yin > 0.0) {
					yin = yin -0.1;
				}//re initialize negative y values
			}
			else yin = 0; // if we are lower than the deadzone set the value to zero -- this avoids "locking in" a non-zero value


			if (stick->GetZ()> (deadzone+.2)|| stick->GetZ()< (-deadzone-.3)){ //z axis deadzone check -- note the z axis needed custom values depending on joystick
				zin = stick->GetZ();
				if (zin<0.0){
					zin = zin +(0.2);
					zin = zin * 0.9; //speed nerf for rotation
				}//re initialize positive z values
				else if (zin > 0.0) {
					zin = zin -(0.3);
					zin = zin * 0.9; //speed nerf for rotation
				}//re initialize negative z values
			}
			else zin = 0; // if we are lower than the deadzone set the value to zero -- this avoids "locking in" a non-zero value

		myRobot->DriveCartesian(xin,yin,zin,0.0); //drive command based on joystick inputs

		// below are the button commands to shoot XV
		if (stick->GetRawButton(1)){
			fire->Set(true);
			reload->Set(false);
			Wait(0.5);
			fire->Set(false);
			reload->Set(true);
		}
		if (stick->GetRawButton(2)){
			if (armed == false){
				armed = true;
				Wait(0.5);
			}
			else if (armed == true){
				armed = false;
				Wait(0.5);
			}
		}
		if (armed == true){
			shoot->Set(1.0);
		}
		else shoot->Set(0.0);

		frc::Scheduler::GetInstance()->Run();

	}

	void TestPeriodic() override {}

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	frc::Command* m_autonomousCommand = nullptr;
	ExampleCommand m_defaultAuto;
	MyAutoCommand m_myAuto;
	frc::SendableChooser<frc::Command*> m_chooser;
};

START_ROBOT_CLASS(Robot)
