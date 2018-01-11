#include <iostream>
#include <memory>
#include <string>

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "Commands/ExampleCommand.h"
#include "CommandBase.h"
#include "WPILib.h"
#include "CANTalon.h"
#include "ADIS16448_IMU.h"

class Robot: public frc::IterativeRobot {

	LiveWindow *lw;
	//XV.Declares window
	CANTalon LF;
	CANTalon LR;
	CANTalon RF;
	CANTalon RR;
	//XV.Declares the wheels
	CANTalon PROP;
	//XV.Declares the firing wheels
	RobotDrive myRobit;
	//XV.Declares the drive
	XboxController rumble;
	//XV.Declares the controller
	Compressor *c = new Compressor(0);
	//XV.Declares the compressor
	Solenoid in;
	Solenoid out;
	//XV.Declares the piston
	ADIS16448_IMU *gyro;
	//XV.Declares the gyro
	Relay spike;
	//XV.Declares the relay
	Servo *servo = new Servo(0);
public:
		Robot():
			lw(NULL),
			//XV.Sets the livewindow
			myRobit(LF, LR, RF, RR),
			LF(10),
			LR(11),
			RF(12),
			RR(13),
			PROP(5),
			rumble(0),
			in(0),
			out(0),
			//XV.Sets the ports
			gyro(),
			spike(0, Relay::kForwardOnly)
			//XV.Sets the gyro and relay
{
			myRobit.SetExpiration(0.1);
			//rumble.SetAxisChannel(Joystick::kTwistAxis, 3);
			myRobit.SetSafetyEnabled(false);
}
	void RobotInit() override {
		gyro = new ADIS16448_IMU;
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {

	}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		/* std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", "Default");
		if (autoSelected == "My Auto") {
			autonomousCommand.reset(new MyAutoCommand());
		}
		else {
			autonomousCommand.reset(new ExampleCommand());
		} */

		autonomousCommand.reset(chooser.GetSelected());

		if (autonomousCommand.get() != nullptr) {
			autonomousCommand->Start();
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
		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
		}
		gyro->Reset();

		servo->Set(.5);
	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
		double x, y, z = 0.0;
		if (rumble.GetRawButton(1))
				{
					gyro->Reset();
				}

		z = rumble.GetRawAxis(2);
		y = rumble.GetRawAxis(1);
		x = rumble.GetRawAxis(0);
		myRobit.MecanumDrive_Cartesian(x, y, z, -gyro->GetAngleZ());
		//rumble.SetRumble(GenericHID(0),1.0);
		if (rumble.GetRawButton(3)){
			servo->SetAngle(160);
		}
		if (rumble.GetRawButton(4))
		{
			servo->SetAngle(5);
		}


	}

	void TestPeriodic() override {
		frc::LiveWindow::GetInstance()->Run();
	}

private:
	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> chooser;
};

START_ROBOT_CLASS(Robot);
