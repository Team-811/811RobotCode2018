package org.usfirst.frc.team811.robot.subsystems;

import org.usfirst.frc.team811.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {

	Joystick joy2 = RobotMap.joystick2;
	DoubleSolenoid gripper = RobotMap.gripperPneumatic;

	public void open() {
		gripper.set(DoubleSolenoid.Value.kReverse);
	}

	public void close() {
		gripper.set(DoubleSolenoid.Value.kForward);
	}

	public void Neutral() {
		// Need to verify what this will do
		gripper.set(DoubleSolenoid.Value.kOff);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
