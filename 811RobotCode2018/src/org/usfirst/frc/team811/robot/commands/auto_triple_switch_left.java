package org.usfirst.frc.team811.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class auto_triple_switch_left extends CommandGroup {

	public auto_triple_switch_left() {
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

		addSequential(new auto_drive_left_switch());
		addSequential(new wait(0.5));
		addSequential(new auto_drive_left_switch_reverse());
		addParallel(new Intake_open());
		addSequential(new auto_drive_switch_cube_pickup_low());
		addSequential(new Intake_close());
		addSequential(new wait(0.5));
		addSequential(new auto_drive_switch_cube_pickup_low_reverse());
		addParallel(new wait_fourbar_switch());
		addSequential(new auto_drive_left_switch());
		addSequential(new Intake_open());
//		addParallel(new wait_fourbar_middle_cube());
//		addSequential(new auto_drive_left_switch_reverse());
//		addParallel(new Intake_open());
//		addSequential(new auto_drive_switch_cube_pickup_high());
//		addSequential(new Intake_close());
//		addSequential(new wait(0.5));
//		addSequential(new auto_drive_switch_cube_pickup_high_reverse());
//		addParallel(new wait_fourbar_switch());
//		addSequential(new auto_drive_left_switch());
//		addSequential(new Intake_open());

	}
}
