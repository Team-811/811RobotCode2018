/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team811.robot;

import org.usfirst.frc.team811.robot.commands.Intake_close;
import org.usfirst.frc.team811.robot.commands.Intake_open;
import org.usfirst.frc.team811.robot.commands.fourbar_climb;
import org.usfirst.frc.team811.robot.commands.fourbar_down;
import org.usfirst.frc.team811.robot.commands.fourbar_switch;
import org.usfirst.frc.team811.robot.commands.fourbar_test;
import org.usfirst.frc.team811.robot.commands.fourbar_up;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI implements Constants {
	//// CREATING BUTTONS
	JoystickButton intake_close;
	JoystickButton intake_open;
	JoystickButton fourbar_climb;
	JoystickButton fourbar_down;
	JoystickButton fourbar_switch;
	JoystickButton fourbar_up;

	public OI() {

		// Operator controller
		intake_close = new JoystickButton(RobotMap.joystick2, INTAKE_BUTTON);
		intake_close.whenReleased(new Intake_close());
		intake_open = new JoystickButton(RobotMap.joystick2, INTAKE_BUTTON);
		intake_open.whenActive(new Intake_open());

		fourbar_up = new JoystickButton(RobotMap.joystick2, FOURBAR_UP_BUTTON);
		fourbar_up.whenPressed(new fourbar_up());
		fourbar_down = new JoystickButton(RobotMap.joystick2, FOURBAR_DOWN_BUTTON);
		fourbar_down.whenPressed(new fourbar_down());
		fourbar_switch = new JoystickButton(RobotMap.joystick2, FOURBAR_SWITCH_BUTTON);
		fourbar_switch.whenPressed(new fourbar_switch());
		fourbar_climb = new JoystickButton(RobotMap.joystick2, FOURBAR_CLIMB_BUTTON);
		fourbar_climb.whenPressed(new fourbar_climb());

		System.out.println("in OI()");

		// Smartdashboard Button
		SmartDashboard.putData("start four bar motor", new fourbar_test());
	}

}
