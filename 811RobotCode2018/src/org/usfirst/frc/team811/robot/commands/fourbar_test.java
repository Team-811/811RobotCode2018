package org.usfirst.frc.team811.robot.commands;

import org.usfirst.frc.team811.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class fourbar_test extends Command {

	public fourbar_test() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.fourBar);
		SmartDashboard.setDefaultNumber("four bar motor value", 0);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double newValue = SmartDashboard.getNumber("four bar motor value", 0);
		Robot.fourBar.setMotorOutput(newValue);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
