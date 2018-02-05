/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team811.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap implements Constants {
	public static Joystick joystick1;
	public static Joystick joystick2;

	public static WPI_TalonSRX drivefrontright;
	public static WPI_TalonSRX drivebackright;
	public static SpeedControllerGroup driveRight;
	public static WPI_TalonSRX drivefrontleft;
	public static WPI_TalonSRX drivebackleft;
	public static SpeedControllerGroup driveLeft;
	public static Encoder driveEncoderLeft;
	public static Encoder driveEncoderRight;
	public static DifferentialDrive driveTrain;
	public static AHRS ahrs;

	public static SpeedController intakeTalon;
	public static DigitalInput intakeLimit;
	// limit switch set in

	public static SpeedController shooterTalon1;
	// public static SpeedController shooterTalon2;
	public static Encoder shooterEncoder;

	public static Talon lifterTalon;
	public static DigitalInput lifterLimitTop;
	public static DigitalInput lifterLimitBottom;
	public static Relay lifterRelay;

	public static NetworkTable visionTable;

	public static Servo servoCam;

	public static DigitalInput climbertopinput;
	public static DigitalInput climberbottominput;

	public void init() {
		// initialize
		joystick1 = new Joystick(1);
		joystick2 = new Joystick(2);

		// Drive Train Motors and Motor Groups
		drivefrontright = new WPI_TalonSRX(FRONT_RIGHT_PORT);
		drivebackright = new WPI_TalonSRX(BACK_RIGHT_PORT);
		driveRight = new SpeedControllerGroup(drivefrontright, drivebackright);
		drivefrontleft = new WPI_TalonSRX(FRONT_LEFT_PORT);
		drivebackleft = new WPI_TalonSRX(BACK_LEFT_PORT);
		driveLeft = new SpeedControllerGroup(drivefrontleft, drivebackleft);
		driveTrain = new DifferentialDrive(driveLeft, driveRight);

		// Drive Train Encoders
		driveEncoderLeft = new Encoder(DRIVE_ENCODER_PORT_LEFT_1, DRIVE_ENCODER_PORT_LEFT_2);
		driveEncoderLeft.setReverseDirection(false);
		driveEncoderLeft.setDistancePerPulse(DRIVE_DISTANCE_PER_PULSE);
		driveEncoderRight = new Encoder(DRIVE_ENCODER_PORT_RIGHT_1, DRIVE_ENCODER_PORT_RIGHT_2);
		driveEncoderRight.setReverseDirection(false);
		driveEncoderRight.setDistancePerPulse(DRIVE_DISTANCE_PER_PULSE);

		// Gyro
		ahrs = new AHRS(SPI.Port.kMXP);

	}
}
