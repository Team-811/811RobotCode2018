package org.usfirst.frc.team811.robot.subsystems;

import org.usfirst.frc.team811.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class RotarySwitch extends Subsystem implements Constants {

	private DigitalInput bit0Input = new DigitalInput(ROTART_SWITCH_BIT0);
	private DigitalInput bit1Input = new DigitalInput(ROTART_SWITCH_BIT1);
	private DigitalInput bit2Input = new DigitalInput(ROTART_SWITCH_BIT2);
	private DigitalInput bit3Input = new DigitalInput(ROTART_SWITCH_BIT3);

	public int switchValue()
	{
		int value = 0;
		if (!bit0Input.get()) value += 1;
		if (!bit1Input.get()) value += 2;
		if (!bit2Input.get()) value += 4;
		if (!bit3Input.get()) value += 8;
		return value;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
