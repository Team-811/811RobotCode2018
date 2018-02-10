package org.usfirst.frc.team811.robot.subsystems;

//import org.usfirst.frc.team811.robot.commands.imagetrack;
import org.usfirst.frc.team811.robot.Constants;
import org.usfirst.frc.team811.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class FourBar extends Subsystem implements Constants, PIDOutput {

	SpeedControllerGroup fourBar = RobotMap.fourBar;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	// strafe command from PID controller
	public void pidWrite(double output) {
		// timer.delay(1);
		SmartDashboard.putNumber("strafe pid output", output);
		SmartDashboard.putNumber("strafe error", fourBarController.getError());
		count++;
		SmartDashboard.putNumber("count", count);
		double errVal = -(m_lastAngle - RobotMap.ahrs.getYaw());
		double P = 0.002;

		rotation = -P * errVal;
		driveTrain.mecanumDrive_Cartesian(output, 0, rotation, 0);
		// Robot.drive.strafe_auto_dist(output);
	}

	AHRS ahrs = RobotMap.ahrs;

	public PIDController fourBarController = RobotMap.fourBarController;

	double rotateToAngleRate;
	double kTolerancePx = 10;

	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system. Note that the */
	/* SmartDashboard in Test mode has support for helping you tune */
	/* controllers by displaying a form where you can enter new P, I, */
	/* and D constants and test the mechanism. */

	@Override
	protected void initDefaultCommand() {

		fourBarController = new PIDController(kP, kI, kD, camSource, (PIDOutput) this);
		// SmartDashboard.putData((NamedSendable) RobotMap.turnController);
		// fourBarController.setInputRange(-1, 130);
		fourBarController.setOutputRange(-1, 1);
		fourBarController.setAbsoluteTolerance(kTolerancePx);
		fourBarController.setContinuous(true);
		fourBarController.setSetpoint(0.0);

		LiveWindow.addActuator("DriveSystem", "fourBarController", fourBarController);

		m_lastAngle = RobotMap.ahrs.getYaw();

	}

	public void strafeTunePID() {
		// double P = SmartDashboard.getNumber("kP");
		// double I = SmartDashboard.getNumber("kI");
		// double D = SmartDashboard.getNumber("kD");

		// fourBarController.setPID(P, I, D);

	}

}
