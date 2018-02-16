/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team811.robot;

import org.usfirst.frc.team811.robot.commands.fourbar_climb;
import org.usfirst.frc.team811.robot.subsystems.FourBar;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap implements Constants {

	// controller
	public static Joystick joystick1;
	public static Joystick joystick2;

	// drive
	public static DifferentialDrive driveTrain;
	public static WPI_TalonSRX drivefrontright;
	public static WPI_TalonSRX drivebackright;
	public static SpeedControllerGroup driveRight;
	public static WPI_TalonSRX drivefrontleft;
	public static WPI_TalonSRX drivebackleft;
	public static SpeedControllerGroup driveLeft;

	// arm
	public static FourBar fourbarArm;
	
	// Intake
	public static DoubleSolenoid gripperPneumatic;

	// Gyro
	public static AHRS ahrs;

	public void init() {
		// initialize

		// controller
		joystick1 = new Joystick(1);
		joystick2 = new Joystick(2);

		// Drive Train Motors and Motor Groups
		drivefrontright = new WPI_TalonSRX(FRONT_RIGHT_PORT);
		drivefrontright.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1);
		drivefrontright.setSensorPhase(true); /* keep sensor and motor in phase */
		drivefrontright.configNeutralDeadband(0.01, 0);

		drivebackright = new WPI_TalonSRX(BACK_RIGHT_PORT);
		drivebackright.configNeutralDeadband(0.01, 0);

		driveRight = new SpeedControllerGroup(drivefrontright, drivebackright);

		drivefrontleft = new WPI_TalonSRX(FRONT_LEFT_PORT);
		drivefrontleft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1);
		drivefrontleft.setSensorPhase(true); /* keep sensor and motor in phase */
		drivefrontleft.configNeutralDeadband(0.01, 0);

		drivebackleft = new WPI_TalonSRX(BACK_LEFT_PORT);
		drivebackleft.configNeutralDeadband(0.01, 0);

		driveLeft = new SpeedControllerGroup(drivefrontleft, drivebackleft);

		driveTrain = new DifferentialDrive(driveLeft, driveRight);

		// Gyro
		ahrs = new AHRS(SPI.Port.kMXP);

		// Intake
		//gripperPneumatic = new DoubleSolenoid(COMPRESSOR_PORT, OPEN_PORT, CLOSE_PORT); // TODO
		
		// fourbar

	}
}
