package org.usfirst.frc.team811.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class FieldData extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public int getSwitchPosition() {
		int position = 0;
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData.charAt(0) == 'L') {
				position = -1; // Switch on left
			} else {
				position = 1; // Switch on right
			}
		}
		return position;
	}

	public int getScalePosition() {
		int position = 0;
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData.charAt(1) == 'L') {
				position = -1; // Switch on left
			} else {
				position = 1; // Switch on right
			}
		}
		return position;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
