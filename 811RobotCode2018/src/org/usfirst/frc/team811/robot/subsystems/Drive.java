package org.usfirst.frc.team811.robot.subsystems;

import org.usfirst.frc.team811.robot.Constants;
import org.usfirst.frc.team811.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class Drive extends Subsystem implements Constants {

	Joystick joy1 = RobotMap.joystick1;
	WPI_TalonSRX frontright = RobotMap.drivefrontright;
	WPI_TalonSRX frontleft = RobotMap.drivefrontleft;
	WPI_TalonSRX backleft = RobotMap.drivebackleft;
	WPI_TalonSRX backright = RobotMap.drivebackright;
	DifferentialDrive driveTrain = RobotMap.driveTrain;
	Encoder driveEncoderLeft = RobotMap.driveEncoder;
	Encoder driveEncoderLeft = RobotMap.driveEncoder;
	// AnalogGyro driveGyro = RobotMap.driveGyro;
	AHRS ahrs = RobotMap.ahrs;
	PIDController turnController = RobotMap.turnController;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void driveWithJoy() {

		double moveVal;
		double turnVal;

		if ((joy1.getRawAxis(FORWARD_DRIVE_AXIS) < .2) && (joy1.getRawAxis(FORWARD_DRIVE_AXIS) > -.2)) {
			moveVal = 0;
		} else {
			moveVal = -joy1.getRawAxis(FORWARD_DRIVE_AXIS);
		}

		if ((joy1.getRawAxis(TURN_DRIVE_AXIS) < .2) && (joy1.getRawAxis(TURN_DRIVE_AXIS) > -.2)) {
			ahrs.reset();
			turnVal = ahrs.getYaw() * -.1;
		} else {
			turnVal = joy1.getRawAxis(TURN_DRIVE_AXIS);
		}

		// driveTrain.arcadeDrive(-1 * moveVal * SPEED_SCALE, turnVal * SPEED_SCALE);
		driveTrain.arcadeDrive(-1 * moveVal * SPEED_SCALE, turnVal * SPEED_SCALE);

		/*
		 * double leftVal = joy1.getRawAxis(FORWARD_DRIVE_AXIS); in case Joe wants
		 * tankdrive double rightVal = joy1.getRawAxis(TURN_DRIVE_AXIS);
		 * driveRobotDrive41.tankDrive(leftVal, rightVal);
		 */
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new drive_w_joysticks());
	}

}
