package org.usfirst.frc.team811.robot.commands;

import org.usfirst.frc.team811.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class auto_drive_switch_approach extends Command {

    public auto_drive_switch_approach() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.motionProfile);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.motionProfile.configureFollower(4, false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.motionProfile.followTrajectory(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.motionProfile.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.motionProfile.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
