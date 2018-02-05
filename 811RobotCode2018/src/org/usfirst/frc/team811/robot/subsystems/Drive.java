package org.usfirst.frc.team811.robot.subsystems;

import org.usfirst.frc.team811.robot.Constants;
import org.usfirst.frc.team811.robot.RobotMap;
import org.usfirst.frc.team811.robot.commands.Drive.drive_w_joystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class Drive extends Subsystem implements Constants {

	Joystick joy1 = RobotMap.joystick1;
	WPI_TalonSRX frontright = RobotMap.drivefrontright;
	WPI_TalonSRX backright = RobotMap.drivebackright;
	SpeedControllerGroup driveRight = RobotMap.driveRight;
	WPI_TalonSRX frontleft = RobotMap.drivefrontleft;
	WPI_TalonSRX backleft = RobotMap.drivebackleft;
	SpeedControllerGroup driveLeft = RobotMap.driveLeft;
	DifferentialDrive driveTrain = RobotMap.driveTrain;
	Encoder driveEncoderLeft = RobotMap.driveEncoderLeft;
	Encoder driveEncoderRight = RobotMap.driveEncoderRight;
	AHRS ahrs = RobotMap.ahrs;

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
		setDefaultCommand(new drive_w_joystick());
	}

}
