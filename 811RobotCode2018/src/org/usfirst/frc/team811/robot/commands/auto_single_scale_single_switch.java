package org.usfirst.frc.team811.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class auto_single_scale_single_switch extends CommandGroup {

	public auto_single_scale_single_switch(boolean leftSide) {
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

		addSequential(new Intake_close());
		addSequential(new wait(0.5));
		addSequential(new auto_drive_scale());
		addSequential(new fourbar_high_scale());
		addSequential(new wait(2));
		addSequential(new auto_drive_approach_scale());
		addSequential(new Intake_open());
		addSequential(new auto_drive_approach_reverse_scale());
		addSequential(new fourbar_down());
		addSequential(new wait(2));
		addSequential(new auto_rotate(180));
		addSequential(new auto_drive_scale_cube_pickup());
		addParallel(new Intake_open());
		addSequential(new Intake_close());
		addSequential(new wait(1));
		addSequential(new fourbar_switch());
		addSequential(new wait(2));
		addSequential(new auto_drive_switch_approach());
		addSequential(new Intake_open());

	}
}
