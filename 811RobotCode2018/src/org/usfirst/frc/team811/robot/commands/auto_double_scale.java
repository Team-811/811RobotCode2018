package org.usfirst.frc.team811.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class auto_double_scale extends CommandGroup {

	public auto_double_scale(boolean LeftSide) {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
		double angle;	

		if (LeftSide) {
			angle = -12;
		} else {
			angle = 12;
		}
		
		

		addSequential(new Intake_close());
		addSequential(new wait(0.5));
		addSequential(new auto_drive_scale(LeftSide));
		addSequential(new fourbar_high_scale());
		addSequential(new wait(2));
		addSequential(new auto_drive_approach_scale());
		addSequential(new Intake_open());
		addSequential(new auto_drive_approach_reverse_scale());
		addSequential(new fourbar_down());
		addSequential(new wait(2));
		addSequential(new auto_rotate(180));
		addSequential(new auto_drive_scale_cube_pickup(LeftSide));
		addParallel(new Intake_open());
		addSequential(new Intake_close());
		addSequential(new wait(1));
		addSequential(new auto_drive_scale_cube_pickup_reverse(LeftSide));
		addSequential(new auto_rotate(angle));
		addSequential(new fourbar_high_scale());
		addSequential(new wait(2));
		addSequential(new auto_drive_approach_scale());
		addSequential(new Intake_open());
	}
}
