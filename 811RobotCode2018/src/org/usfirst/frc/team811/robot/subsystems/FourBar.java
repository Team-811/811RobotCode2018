package org.usfirst.frc.team811.robot.subsystems;

//import org.usfirst.frc.team811.robot.commands.imagetrack;
import org.usfirst.frc.team811.robot.Constants;
import org.usfirst.frc.team811.robot.RobotMap;
import org.usfirst.frc.team811.robot.commands.fourbar_w_joystick;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class FourBar extends Subsystem implements Constants, PIDSource, PIDOutput {

	private double MAX_SPEED = 0.45;

	// PIDSource begin implementation
	PIDSourceType type = PIDSourceType.kDisplacement;

	@Override
	public void setPIDSourceType(PIDSourceType pidSourceType) {
		type = pidSourceType;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}

	@Override
	public double pidGet() {
		return (double) leftTalon.getSelectedSensorPosition(0);
	}
	// PIDSource end implementation

	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system. Note that the */
	/* SmartDashboard in Test mode has support for helping you tune */
	/* controllers by displaying a form where you can enter new P, I, */
	/* and D constants and test the mechanism. */

	private double kP = 0.0004;
	private double kI = 0.00;
	private double kD = 0.0002;
	private double kF = 0.00; // look this up
	private double kTolerancePx = 100;
	private double kParkPosition = 0.0;
	private double kJoystickMultiplier = 10;

	Joystick joy2 = RobotMap.joystick2;

	// four bar
	private WPI_TalonSRX leftTalon;
	private WPI_TalonSRX rightTalon;
	private SpeedControllerGroup talonGroup;
	private PIDController fourBarController;
	private DigitalInput bottomLimitSwitch;

	private boolean isParking = true;

	public FourBar(int leftPort, int rightPort) {

		// left talon will be wired to the sensor
		// TODO: verify with actual robot
		leftTalon = new WPI_TalonSRX(leftPort);
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1);
		leftTalon.setSensorPhase(false); /* keep sensor and motor in phase */
		leftTalon.configNeutralDeadband(0.01, 0);
		leftTalon.setSelectedSensorPosition(0, 0, 5);

		rightTalon = new WPI_TalonSRX(rightPort);
		rightTalon.configNeutralDeadband(0.01, 0);

		// NOTE: the motors on the fourbar spin in opposite directions
		invertMotors();

		talonGroup = new SpeedControllerGroup(leftTalon, rightTalon);

		setBrakeModeOn(true);

		// set park to true before enabling the PID controller for the first time
		isParking = true;
		fourBarController = new PIDController(kP, kI, kD, this /* as PIDSource */, this /* as PIDOutput */);

		fourBarController.setOutputRange(-0.25, 0.25);
		fourBarController.setAbsoluteTolerance(kTolerancePx);
		fourBarController.setContinuous(false);
		fourBarController.setSetpoint(0.0);
		fourBarController.disable();
	}

	private void setBrakeModeOn(boolean brakeOn) {
		NeutralMode mode = (brakeOn ? NeutralMode.Brake : NeutralMode.Coast);
		rightTalon.setNeutralMode(mode);
		leftTalon.setNeutralMode(mode);
	}

	private void invertMotors() {
		leftTalon.setInverted(true);
		rightTalon.setInverted(false);
	}

	public void setMotorOutput(double motorCommand) {
		// command needs to be between -1 and 1
		if (motorCommand > MAX_SPEED) {
			motorCommand = MAX_SPEED;
		} else if (motorCommand < -MAX_SPEED) {
			motorCommand = -MAX_SPEED;
		}

		SmartDashboard.putNumber("motorCommand", motorCommand);
		talonGroup.set(motorCommand);
	}

	public void fourBarWithJoy() {

		double setpointVal = fourBarController.getSetpoint();

		if ((joy2.getRawAxis(FOURBAR_AXIS) > .2) || (joy2.getRawAxis(FOURBAR_AXIS) < -.2)) {
			setpointVal = setpointVal + (joy2.getRawAxis(FOURBAR_AXIS) * kJoystickMultiplier);
			fourBarController.setSetpoint(setpointVal);
		}

	}

	// setting a position will use the PID controller to reach that
	public void setPostion(double desiredEncoderPosition) {

		double delta = Math.abs(fourBarController.getSetpoint() - desiredEncoderPosition);
		if (delta > 1) {
			fourBarController.setSetpoint(desiredEncoderPosition);
			fourBarController.enable();

			isParking = (desiredEncoderPosition - kParkPosition) < 1;
		}
	}

	// PID controller output
	public void pidWrite(double output) {
		SmartDashboard.putNumber("strafe pid output", output);

		// Take the output of the PID loop and add the offset to hold position
		double command = output + getHoldingCommand();

		// SmartDashboard.putNumber("strafe error", fourBarController.getError());
		if (isParking && bottomLimitSwitch.get()) {
			// if parking and the limit switch is activated - the arm is all the way down
			// and the motor command 0
			command = 0.0;
			fourBarController.disable();

		}
		setMotorOutput(command);
	}

	private double getHoldingCommand() {
		// for now this is just a constant
		return 0.27;
	}

	@Override
	protected void initDefaultCommand() {
		// LiveWindow.addActuator("DriveSystem", "fourBarController",
		// fourBarController);MAX_SPEED

		setDefaultCommand(new fourbar_w_joystick());
		// setDefaultCommand(new fourbar_test());
	}

	public void tunePID() {
		// double P = SmartDashboard.getNumber("kP");
		// double I = SmartDashboard.getNumber("kI");
		// double D = SmartDashboard.getNumber("kD");

		// fourBarController.setPID(P, I, D);

	}

	public void encoderValue() {
		SmartDashboard.putNumber("Four Bar Encoder", leftTalon.getSelectedSensorPosition(0));
	}

}
