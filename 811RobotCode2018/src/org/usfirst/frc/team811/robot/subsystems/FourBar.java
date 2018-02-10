package org.usfirst.frc.team811.robot.subsystems;

//import org.usfirst.frc.team811.robot.commands.imagetrack;
import org.usfirst.frc.team811.robot.Constants;
import org.usfirst.frc.team811.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
		return (double)leftTalon.getSelectedSensorPosition(0);
	}
	// PIDSource end implementation
	
	private double kP = 0.03;
	private double kI = 0.00;
	private double kD = 0.01;
	private double kF = 0.00;  // look this up
	private double kTolerancePx = 10;


	// four bar
	private WPI_TalonSRX leftTalon;
	private WPI_TalonSRX rightTalon;
	private SpeedControllerGroup talonGroup;
	private PIDController fourBarController;
	
	public FourBar(){
		
		PIDController fourBarController = new PIDController(kP, kI, kD, 
				this /* as PIDSource */, 
				this /* as PIDOutput */);

	}

	public void init(int leftPort, int rightPort)
	{
		fourBarController.setOutputRange(-1, 1);
		fourBarController.setAbsoluteTolerance(kTolerancePx);
		fourBarController.setContinuous(true);
		fourBarController.setSetpoint(0.0);

		// Four Bar
		leftTalon = new WPI_TalonSRX(leftPort); // TODO
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 1);
		leftTalon.setSensorPhase(true); /* keep sensor and motor in phase */
		leftTalon.configNeutralDeadband(0.01, 0);
	
		rightTalon = new WPI_TalonSRX(rightPort);
		rightTalon.configNeutralDeadband(0.01, 0);

		talonGroup = new SpeedControllerGroup(leftTalon, rightTalon);		
	}

	public void setMotorOutput(double motorCommand)
	{
		// command needs to be between -1 and 1
		if (motorCommand > 1.0) {
			motorCommand = 1.0;
		}
		else if (motorCommand < -1.0){
			motorCommand = -1.0;
		}
		
		talonGroup.set(motorCommand);
	}

	// PID controller output
	public void pidWrite(double output) {
//		SmartDashboard.putNumber("strafe pid output", output);
		
		// Take the output of the PID loop and add the offset to hold position
		double command = output + getHoldingCommand();
				
//		SmartDashboard.putNumber("strafe error", fourBarController.getError());

		setMotorOutput(command);
	}
	

	private double getHoldingCommand()
	{
		// for now this is just a constant
		return 0.0;
	}
	
	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system. Note that the */
	/* SmartDashboard in Test mode has support for helping you tune */
	/* controllers by displaying a form where you can enter new P, I, */
	/* and D constants and test the mechanism. */

	@Override
	protected void initDefaultCommand() {
		LiveWindow.addActuator("DriveSystem", "fourBarController", fourBarController);
	}

	public void tunePID() {
		// double P = SmartDashboard.getNumber("kP");
		// double I = SmartDashboard.getNumber("kI");
		// double D = SmartDashboard.getNumber("kD");

		// fourBarController.setPID(P, I, D);

	}

}
