package org.usfirst.frc.team811.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class auto_triple_switch_right extends CommandGroup {

	public auto_triple_switch_right() {
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

		addSequential(new auto_drive_right_switch());
		addSequential(new wait(0.5));
		addSequential(new auto_drive_right_switch_reverse());
		addSequential(new auto_drive_switch_cube_pickup_low());
		addParallel(new Intake_open());
		addSequential(new Intake_close());
		addSequential(new wait(0.5));
		addSequential(new auto_drive_switch_cube_pickup_low_reverse());
		addSequential(new auto_drive_right_switch());
		addParallel(new fourbar_switch());
		addSequential(new Intake_open());
		addSequential(new auto_drive_right_switch_reverse());
		addParallel(new fourbar_middle_cube());
		addSequential(new auto_drive_switch_cube_pickup_high());
		addParallel(new Intake_open());
		addSequential(new Intake_close());
		addSequential(new wait(0.5));
		addSequential(new auto_drive_switch_cube_pickup_high_reverse());
		addSequential(new auto_drive_right_switch());
		addParallel(new fourbar_switch());
		addSequential(new Intake_open());

	}
}
