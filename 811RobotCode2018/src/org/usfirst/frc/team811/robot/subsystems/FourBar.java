package org.usfirst.frc.team811.robot.subsystems;

//import org.usfirst.frc.team811.robot.commands.imagetrack;
import org.usfirst.frc.team811.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
	private double kP = 0.03;
	private double kI = 0.00;
	private double kD = 0.01;
	private double kF = 0.00; // look this up
	private double kTolerancePx = 10;
	private double kParkPosition = 0.0;

	// four bar
	private WPI_TalonSRX leftTalon;
	private WPI_TalonSRX rightTalon;
	private SpeedControllerGroup talonGroup;
	private PIDController fourBarController;

	private boolean isParking = true;

	public FourBar(int leftPort, int rightPort) {

		// left talon will be wired to the sensor
		// TODO: verify with actual robot
		leftTalon = new WPI_TalonSRX(leftPort);
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 1);
		leftTalon.setSensorPhase(true); /* keep sensor and motor in phase */
		leftTalon.configNeutralDeadband(0.01, 0);
		// leftTalon.setSelectedSensorPosition(0, 0, 5);

		rightTalon = new WPI_TalonSRX(rightPort);
		rightTalon.configNeutralDeadband(0.01, 0);

		// NOTE: the motors on the fourbar spin in opposite directions
		// invertMotors();

		talonGroup = new SpeedControllerGroup(leftTalon, rightTalon);

		// setBrakeModeOn(true);

		// set park to true before enabling the PID controller for the first time
		isParking = true;
		fourBarController = new PIDController(kP, kI, kD, this /* as PIDSource */, this /* as PIDOutput */);

		fourBarController.setOutputRange(-1, 1);
		fourBarController.setAbsoluteTolerance(kTolerancePx);
		fourBarController.setContinuous(false);
		fourBarController.setSetpoint(0.0);
		// fourBarController.enable();
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
		if (motorCommand > 1.0) {
			motorCommand = 1.0;
		} else if (motorCommand < -1.0) {
			motorCommand = -1.0;
		}

		talonGroup.set(motorCommand);
	}

	// setting a position will use the PID controller to reach that
	public void setPostion(double desiredEncoderPosition) {

		fourBarController.setSetpoint(desiredEncoderPosition);

		isParking = (desiredEncoderPosition - kParkPosition) < 0.1;
	}

	// Make the arm safe and deactive PID control
	public void park() {
		// set the position to 0
		setPostion(kParkPosition);
	}

	// PID controller output
	public void pidWrite(double output) {
		// SmartDashboard.putNumber("strafe pid output", output);

		// Take the output of the PID loop and add the offset to hold position
		double command = output + getHoldingCommand();

		// SmartDashboard.putNumber("strafe error", fourBarController.getError());
		if (isParking && (output < 0.001)) {
			// if parking and the output is 0 - the arm is all the way down
			// and the motor command 0
			command = 0.0;
		}
		setMotorOutput(command);
	}

	private double getHoldingCommand() {
		// for now this is just a constant
		return 0.0;
	}

	@Override
	protected void initDefaultCommand() {
		// LiveWindow.addActuator("DriveSystem", "fourBarController",
		// fourBarController);
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
