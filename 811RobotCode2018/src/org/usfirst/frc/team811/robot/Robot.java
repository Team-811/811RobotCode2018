/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team811.robot;

import org.usfirst.frc.team811.robot.commands.auto_drive_left_switch;
import org.usfirst.frc.team811.robot.commands.auto_drive_right_switch;
import org.usfirst.frc.team811.robot.subsystems.Drive;
import org.usfirst.frc.team811.robot.subsystems.FieldData;
import org.usfirst.frc.team811.robot.subsystems.FourBar;
import org.usfirst.frc.team811.robot.subsystems.Intake;
import org.usfirst.frc.team811.robot.subsystems.MotionProfile;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot implements Constants {

	public static OI oi;
	public static RobotMap robotMap;

	public static Drive drive;
	public static Intake intake;
	public static FourBar fourBar;
	public static MotionProfile motionProfile;
	public static FieldData fieldData;

	Command m_autonomousCommand;
	//SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		robotMap = new RobotMap();
		robotMap.init();

		drive = new Drive();
		intake = new Intake();
		fourBar = new FourBar(FOURBAR_LEFT_PORT, FOURBAR_RIGHT_PORT);
		motionProfile = new MotionProfile();
		fieldData = new FieldData();

		RobotMap.drivefrontleft.setSelectedSensorPosition(0, 0, 3);
		RobotMap.drivefrontright.setSelectedSensorPosition(0, 0, 3);

		oi = new OI(); // has to go last
		// chooser.addObject("My Auto", new MyAutoCommand());
		//SmartDashboard.putData("Auto mode", m_chooser);

		SmartDashboard.setDefaultNumber("PID Setpoint", 0);

		motionProfile.generateLeftSwitchTrajectory();
		motionProfile.generateRightSwitchTrajectory();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		// m_autonomousCommand = m_chooser.getSelected();

		int switchSide = Robot.fieldData.getSwitchPosition();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		 * switch(autoSelected) { case "My Auto": autonomousCommand = new
		 * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
		 * ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)

		if (switchSide == -1) {
			m_autonomousCommand = new auto_drive_left_switch();
			m_autonomousCommand.start();
		} else {
			m_autonomousCommand = new auto_drive_right_switch();
			m_autonomousCommand.start();
		}

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		fourBar.encoderValue();
		SmartDashboard.putNumber("Pid Output", fourBar.pidGet());
		//double setpoint = SmartDashboard.getNumber("PID Setpoint", 0);
		//fourBar.setPostion(setpoint);
		SmartDashboard.putNumber("Left Drive", RobotMap.drivefrontleft.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right Drive", RobotMap.drivefrontright.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Gyro", RobotMap.ahrs.getYaw());
		SmartDashboard.putNumber("Rotary Switch", RobotMap.autoSelect.switchValue());
		fourBar.encoderValue();
		
		if (fourBar.encoderCount() >= 6000) {
			RobotMap.SpeedCutoff = ENCODER_LOW;
		} else {
			RobotMap.SpeedCutoff = ENCODER_HIGH;
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
