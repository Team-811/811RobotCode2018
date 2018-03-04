package org.usfirst.frc.team811.robot.subsystems;

import org.usfirst.frc.team811.robot.Constants;
import org.usfirst.frc.team811.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 *
 */
public class MotionProfile extends Subsystem implements Constants, PIDSource, PIDOutput {

	WPI_TalonSRX frontright = RobotMap.drivefrontright;
	WPI_TalonSRX backright = RobotMap.drivebackright;
	SpeedControllerGroup driveRight = RobotMap.driveRight;
	WPI_TalonSRX frontleft = RobotMap.drivefrontleft;
	WPI_TalonSRX backleft = RobotMap.drivebackleft;
	SpeedControllerGroup driveLeft = RobotMap.driveLeft;
	DifferentialDrive driveTrain = RobotMap.driveTrain;
	AHRS ahrs = RobotMap.ahrs;

	private PIDController rotateController;

	private double MAX_SPEED = 0.5;

	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system. Note that the */
	/* SmartDashboard in Test mode has support for helping you tune */
	/* controllers by displaying a form where you can enter new P, I, */
	/* and D constants and test the mechanism. */

	private static double rotateKP = 0.003;
	private static double rotateKI = 0.00;
	private static double rotateKD = 0.001;
	private static double rotateKTolerance = 100.0;

	EncoderFollower leftFollower;
	EncoderFollower rightFollower;
	TankModifier modifier;
	TankModifier modifierLeft;
	TankModifier modifierRight;
	TankModifier modifier1;

	double l;
	double r;

	static double max_velocity = 0.45;
	static double max_acceleration = 0.5;
	static double max_jerk = 60.0;
	static double wheel_diameter = 0.15;
	static double wheel_base_distance = 0.62;
	static int encoder_rotation = 1060;
	static double kD = 0.2;
	static double kP = 1;
	static double acceleration_gain = 0.0;
	// TODO
	static double absolute_max_velocity = 3.612;
	static double gyro_correction_power = 0.7;
	static double yDirectionCorrection = 0.9;

	int leftEncoderStartingPosition;
	int rightEncoderStartingPosition;

	static int occuranceNumber = 0;
	static int occuranceTolerance = 10;

	public MotionProfile() {
		rotateController = new PIDController(rotateKP, rotateKI, rotateKD, this, this);
		rotateController.setOutputRange(-0.5, 0.5);
		rotateController.setAbsoluteTolerance(rotateKTolerance);
		rotateController.setContinuous(true);
		rotateController.setSetpoint(0.0);
		rotateController.disable();
	}

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
		return (double) ahrs.getYaw();
	}
	// PIDSource end implementation

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	// PID Controller

	public void pidWrite(double output) {
		// SmartDashboard.putNumber("strafe pid output", output);

		// Take the output of the PID loop and add the offset to hold position
		double command = output;

		setRotate(command);
	}

	public void setRotate(double motorCommand) {
		// command needs to be between -1 and 1
		if (motorCommand > MAX_SPEED) {
			motorCommand = MAX_SPEED;
		} else if (motorCommand < -MAX_SPEED) {
			motorCommand = -MAX_SPEED;
		}

		// SmartDashboard.putNumber("motorCommand", motorCommand);
		driveTrain.arcadeDrive(0, motorCommand);
	}

	public void setPostion(double desiredAngle) {

		double delta = Math.abs(rotateController.getSetpoint() - desiredAngle);
		if (delta > 0.001) {
			rotateController.setSetpoint(desiredAngle);

			// isParking = (desiredEncoderPosition - kParkPosition) < 0.1;
		}
	}

	// Path Finder

	public void configureFollower(int Side) {

		if (Side == -1) {
			modifier = modifierLeft;
		} else {
			modifier = modifierRight;
		}

		ahrs.reset();

		leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
		rightFollower = new EncoderFollower(modifier.getRightTrajectory());

		leftEncoderStartingPosition = frontleft.getSelectedSensorPosition(0);
		rightEncoderStartingPosition = frontright.getSelectedSensorPosition(0);
		leftFollower.configureEncoder(leftEncoderStartingPosition, encoder_rotation, wheel_diameter);
		rightFollower.configureEncoder(rightEncoderStartingPosition, encoder_rotation, wheel_diameter);
		leftFollower.configurePIDVA(kP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);
		rightFollower.configurePIDVA(kP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);

	}

	public void configureFollower1() {

		ahrs.reset();

		leftFollower = new EncoderFollower(modifier1.getLeftTrajectory());
		rightFollower = new EncoderFollower(modifier1.getRightTrajectory());

		leftEncoderStartingPosition = frontleft.getSelectedSensorPosition(0);
		rightEncoderStartingPosition = frontright.getSelectedSensorPosition(0);
		leftFollower.configureEncoder(leftEncoderStartingPosition, encoder_rotation, wheel_diameter);
		rightFollower.configureEncoder(rightEncoderStartingPosition, encoder_rotation, wheel_diameter);
		leftFollower.configurePIDVA(kP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);
		rightFollower.configurePIDVA(kP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);

	}

	public void configureFollower() {

		ahrs.reset();

		leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
		rightFollower = new EncoderFollower(modifier.getRightTrajectory());

		leftEncoderStartingPosition = frontleft.getSelectedSensorPosition(0);
		rightEncoderStartingPosition = frontright.getSelectedSensorPosition(0);
		leftFollower.configureEncoder(leftEncoderStartingPosition, encoder_rotation, wheel_diameter);
		rightFollower.configureEncoder(rightEncoderStartingPosition, encoder_rotation, wheel_diameter);
		leftFollower.configurePIDVA(kP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);
		rightFollower.configurePIDVA(kP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);

	}

	public void followTrajectory(boolean reverse) {

		int changeDirection;

		if (reverse) {
			changeDirection = -1;
		} else {
			changeDirection = 1;
		}

		int leftEncoderPosition = leftEncoderStartingPosition
				+ Math.abs(leftEncoderStartingPosition - frontleft.getSelectedSensorPosition(0));
		int rightEncoderPosition = rightEncoderStartingPosition
				+ Math.abs(rightEncoderStartingPosition - frontright.getSelectedSensorPosition(0));

		l = leftFollower.calculate(leftEncoderPosition);
		r = rightFollower.calculate(rightEncoderPosition);

		double gyro_heading = ahrs.getYaw() * -1; // counter clockwise rotation should be positive
		double desired_heading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		double turn = gyro_correction_power * (-1.0 / 80.0) * angleDifference;

		driveTrain.tankDrive((l * changeDirection) + turn, (r * changeDirection) - turn);
		// driveTrain.tankDrive((l * changeDirection), (r * changeDirection) );

	}

	public void reset() {
		driveTrain.tankDrive(0, 0);
	}

	public Boolean isFinished() {
		if (l == 0 && r == 0) {

			occuranceNumber++;

			if (occuranceNumber >= occuranceTolerance) {
				occuranceNumber = 0;
				return true;
			} else {
				return false;
			}

		} else {
			return false;
		}
	}

	// different paths
	public static void printTrajectory(Trajectory trajectory, String name) {
		System.out.printf("\t\t// segmments for %s\n", name);
		System.out.print("\t\tTrajectory.Segment[] segs = new Trajectory.Segment[] {\n");

		for (int i = 0; i < trajectory.length(); i++) {
			Trajectory.Segment seg = trajectory.get(i);

			System.out.printf("\t\t\tnew Trajectory.Segment(%f,%f,%f,%f,%f,%f,%f,%f),\n", seg.dt, seg.x, seg.y,
					seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		}
		System.out.print("\t\t};");
	}

	private static TankModifier generateTrajectory(Waypoint[] points, String name) {

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);

		Trajectory trajectory = Pathfinder.generate(points, config);
		return new TankModifier(trajectory).modify(wheel_base_distance);
	}

	private static void generateAndPrintTrajectory(Waypoint[] points, String name) {

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);

		Trajectory trajectory = Pathfinder.generate(points, config);
		printTrajectory(trajectory, name);
	}

	public static void g1() {
		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(4, 0, 0) };

		generateAndPrintTrajectory(points, "staight");
	}

	public static void g2() {
		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0),
				new Waypoint(1.25, 1.37 * yDirectionCorrection, Pathfinder.d2r(50)),
				new Waypoint(2.74, 2.66 * yDirectionCorrection, 0) };

		generateAndPrintTrajectory(points, "left switch");
	}

	public static void g3() {
		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(2.74, 0 * yDirectionCorrection, 0) };

		generateAndPrintTrajectory(points, "right switch");
	}

	public void generateDriveStraightTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5.5, 0 * yDirectionCorrection, 0) };

		modifier = generateTrajectory(points, "straight");
	}

	public void generateLeftSwitchTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0),
				// new Waypoint(1.3, 1.37 * yDirectionCorrection, Pathfinder.d2r(50)),
				new Waypoint(3, 2.66 * yDirectionCorrection, 0) };

		modifierLeft = generateTrajectory(points, "left switch");
	}

	public void generateRightSwitchTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(2.74, 0 * yDirectionCorrection, 0) };

		modifierRight = generateTrajectory(points, "right switch");
	}

	/*
	 * public void generateRightSwitchToCubePickupTrajectory() {
	 * 
	 * Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0
	 * * yDirectionCorrection, 0) };
	 * 
	 * generateTrajectory(points, modifier, "right cub pickup"); }
	 * 
	 * public void generateLeftSwitchToCubePickupTrajectory() { Waypoint[] points =
	 * new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0 *
	 * yDirectionCorrection, 0) };
	 * 
	 * generateTrajectory(points, modifier, "left cube pickup"); }
	 * 
	 * public void generateCubePickupTrajectory() {
	 * 
	 * Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0
	 * * yDirectionCorrection, 0) };
	 * 
	 * generateTrajectory(points, modifier, "cube pickup"); }
	 */

	// TODO
	public void generateScaleLeftTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0 * yDirectionCorrection, 0) };

		modifier = generateTrajectory(points, "scale");
	}

	public void generateScaleRightTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0 * yDirectionCorrection, 0) };

		modifier = generateTrajectory(points, "scale");
	}

	public void generateApproachScaleTrajectory() {

		double current_max = max_velocity;
		max_velocity = 0.25;

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(2, 0 * yDirectionCorrection, 0) };

		modifier1 = generateTrajectory(points, "approach");

		max_velocity = current_max;

	}

	/*
	 * // segmments for staight Trajectory.Segment[] segs = new Trajectory.Segment[]
	 * { new
	 * Trajectory.Segment(0.050000,0.000585,0.000000,0.000625,0.025000,0.500000,10.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.002460,0.000000,0.002500,0.050000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.005585,0.000000,0.005625,0.075000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.009960,0.000000,0.010000,0.100000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.015585,0.000000,0.015625,0.125000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.022460,0.000000,0.022500,0.150000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.030585,0.000000,0.030625,0.175000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.039960,0.000000,0.040000,0.200000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.050585,0.000000,0.050625,0.225000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.062460,0.000000,0.062500,0.250000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.075585,0.000000,0.075625,0.275000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.089960,0.000000,0.090000,0.300000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.105585,0.000000,0.105625,0.325000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.122460,0.000000,0.122500,0.350000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.140585,0.000000,0.140625,0.375000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.159960,0.000000,0.160000,0.400000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.180585,0.000000,0.180625,0.425000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.202460,0.000000,0.202500,0.450000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.225585,0.000000,0.225625,0.475000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.249960,0.000000,0.250000,0.500000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.274960,0.000000,0.275000,0.500000,0.000000,-10.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.299960,0.000000,0.300000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.324960,0.000000,0.325000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.349960,0.000000,0.350000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.374960,0.000000,0.375000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.399960,0.000000,0.400000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.424960,0.000000,0.425000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.449960,0.000000,0.450000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.474960,0.000000,0.475000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.499960,0.000000,0.500000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.524960,0.000000,0.525000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.549960,0.000000,0.550000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.574960,0.000000,0.575000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.599960,0.000000,0.600000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.624960,0.000000,0.625000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.649960,0.000000,0.650000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.674960,0.000000,0.675000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.699960,0.000000,0.700000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.724960,0.000000,0.725000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.749960,0.000000,0.750000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.774960,0.000000,0.775000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.799960,0.000000,0.800000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.824960,0.000000,0.825000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.849960,0.000000,0.850000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.874960,0.000000,0.875000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.899960,0.000000,0.900000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.924960,0.000000,0.925000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.949960,0.000000,0.950000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.974960,0.000000,0.975000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.999960,0.000000,1.000000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.024960,0.000000,1.025000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.049960,0.000000,1.050000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.074960,0.000000,1.075000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.099960,0.000000,1.100000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.124960,0.000000,1.125000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.149960,0.000000,1.150000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.174960,0.000000,1.175000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.199960,0.000000,1.200000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.224960,0.000000,1.225000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.249960,0.000000,1.250000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.274960,0.000000,1.275000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.299960,0.000000,1.300000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.324960,0.000000,1.325000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.349960,0.000000,1.350000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.374960,0.000000,1.375000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.399960,0.000000,1.400000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.424960,0.000000,1.425000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.449960,0.000000,1.450000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.474960,0.000000,1.475000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.499960,0.000000,1.500000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.524960,0.000000,1.525000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.549960,0.000000,1.550000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.574960,0.000000,1.575000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.599960,0.000000,1.600000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.624960,0.000000,1.625000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.649960,0.000000,1.650000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.674960,0.000000,1.675000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.699960,0.000000,1.700000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.724960,0.000000,1.725000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.749960,0.000000,1.750000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.774960,0.000000,1.775000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.799960,0.000000,1.800000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.824960,0.000000,1.825000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.849960,0.000000,1.850000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.874960,0.000000,1.875000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.899960,0.000000,1.900000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.924960,0.000000,1.925000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.949960,0.000000,1.950000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.974960,0.000000,1.975000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.999960,0.000000,2.000000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.024960,0.000000,2.025000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.049960,0.000000,2.050000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.074960,0.000000,2.075000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.099960,0.000000,2.100000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.124960,0.000000,2.125000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.149960,0.000000,2.150000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.174960,0.000000,2.175000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.199960,0.000000,2.200000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.224960,0.000000,2.225000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.249960,0.000000,2.250000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.274960,0.000000,2.275000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.299960,0.000000,2.300000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.324960,0.000000,2.325000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.349960,0.000000,2.350000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.374960,0.000000,2.375000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.399960,0.000000,2.400000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.424960,0.000000,2.425000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.449960,0.000000,2.450000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.474960,0.000000,2.475000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.499960,0.000000,2.500000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.524960,0.000000,2.525000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.549960,0.000000,2.550000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.574960,0.000000,2.575000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.599960,0.000000,2.600000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.624960,0.000000,2.625000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.649960,0.000000,2.650000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.674960,0.000000,2.675000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.699960,0.000000,2.700000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.724960,0.000000,2.725000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.749960,0.000000,2.750000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.774960,0.000000,2.775000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.799960,0.000000,2.800000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.824960,0.000000,2.825000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.849960,0.000000,2.850000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.874960,0.000000,2.875000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.899960,0.000000,2.900000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.924960,0.000000,2.925000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.949960,0.000000,2.950000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.974960,0.000000,2.975000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.999960,0.000000,3.000000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.024960,0.000000,3.025000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.049960,0.000000,3.050000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.074960,0.000000,3.075000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.099960,0.000000,3.100000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.124960,0.000000,3.125000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.149960,0.000000,3.150000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.174960,0.000000,3.175000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.199960,0.000000,3.200000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.224960,0.000000,3.225000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.249960,0.000000,3.250000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.274960,0.000000,3.275000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.299960,0.000000,3.300000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.324960,0.000000,3.325000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.349960,0.000000,3.350000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.374960,0.000000,3.375000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.399960,0.000000,3.400000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.424960,0.000000,3.425000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.449960,0.000000,3.450000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.474960,0.000000,3.475000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.499960,0.000000,3.500000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.524960,0.000000,3.525000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.549960,0.000000,3.550000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.574960,0.000000,3.575000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.599960,0.000000,3.600000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.624960,0.000000,3.625000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.649960,0.000000,3.650000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.674960,0.000000,3.675000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.699960,0.000000,3.700000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.724960,0.000000,3.725000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.749960,0.000000,3.750000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.774336,0.000000,3.774376,0.475040,-0.499200,-9.
	 * 984000,0.000000), new
	 * Trajectory.Segment(0.050000,3.797463,0.000000,3.797503,0.450040,-0.500000,-0.
	 * 016000,0.000000), new
	 * Trajectory.Segment(0.050000,3.819340,0.000000,3.819380,0.425040,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.839967,0.000000,3.840007,0.400040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.859344,0.000000,3.859384,0.375040,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.877471,0.000000,3.877511,0.350040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.894348,0.000000,3.894388,0.325040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.909975,0.000000,3.910015,0.300040,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.924352,0.000000,3.924392,0.275040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.937479,0.000000,3.937519,0.250040,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.949356,0.000000,3.949396,0.225040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.959983,0.000000,3.960023,0.200040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.969360,0.000000,3.969400,0.175040,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.977487,0.000000,3.977527,0.150040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.984364,0.000000,3.984404,0.125040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.989991,0.000000,3.990031,0.100040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.994368,0.000000,3.994408,0.075040,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.997495,0.000000,3.997535,0.050040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.999372,0.000000,3.999412,0.025040,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,3.999999,0.000000,4.000039,0.000040,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,4.000000,0.000000,4.000040,0.000000,-0.000800,9.
	 * 984000,0.000000), new
	 * Trajectory.Segment(0.050000,4.000000,0.000000,4.000040,0.000000,0.000000,0.
	 * 016000,0.000000), }; // segmments for left switch Trajectory.Segment[] segs =
	 * new Trajectory.Segment[] { new
	 * Trajectory.Segment(0.050000,0.000601,0.000000,0.000625,0.025000,0.500000,10.
	 * 000000,0.000474), new
	 * Trajectory.Segment(0.050000,0.002476,0.000002,0.002500,0.050000,0.500000,0.
	 * 000000,0.001956), new
	 * Trajectory.Segment(0.050000,0.005601,0.000012,0.005625,0.075000,0.500000,-0.
	 * 000000,0.004435), new
	 * Trajectory.Segment(0.050000,0.009976,0.000039,0.010000,0.100000,0.500000,0.
	 * 000000,0.007928), new
	 * Trajectory.Segment(0.050000,0.015601,0.000097,0.015625,0.125000,0.500000,-0.
	 * 000000,0.012457), new
	 * Trajectory.Segment(0.050000,0.022475,0.000202,0.022500,0.150000,0.500000,0.
	 * 000000,0.018049), new
	 * Trajectory.Segment(0.050000,0.030598,0.000375,0.030625,0.175000,0.500000,0.
	 * 000000,0.024741), new
	 * Trajectory.Segment(0.050000,0.039969,0.000644,0.040000,0.200000,0.500000,0.
	 * 000000,0.032573), new
	 * Trajectory.Segment(0.050000,0.050587,0.001038,0.050625,0.225000,0.500000,-0.
	 * 000000,0.041597), new
	 * Trajectory.Segment(0.050000,0.062449,0.001592,0.062500,0.250000,0.500000,0.
	 * 000000,0.051867), new
	 * Trajectory.Segment(0.050000,0.075552,0.002348,0.075625,0.275000,0.500000,0.
	 * 000000,0.063449), new
	 * Trajectory.Segment(0.050000,0.089892,0.003352,0.090000,0.300000,0.500000,-0.
	 * 000000,0.076415), new
	 * Trajectory.Segment(0.050000,0.105462,0.004657,0.105625,0.325000,0.500000,0.
	 * 000000,0.090843), new
	 * Trajectory.Segment(0.050000,0.122255,0.006322,0.122500,0.350000,0.500000,-0.
	 * 000000,0.106821), new
	 * Trajectory.Segment(0.050000,0.140258,0.008412,0.140625,0.375000,0.500000,0.
	 * 000000,0.124442), new
	 * Trajectory.Segment(0.050000,0.159459,0.011002,0.160000,0.400000,0.500000,0.
	 * 000000,0.143805), new
	 * Trajectory.Segment(0.050000,0.179839,0.014173,0.180625,0.425000,0.500000,-0.
	 * 000000,0.165010), new
	 * Trajectory.Segment(0.050000,0.201373,0.018014,0.202500,0.450000,0.500000,0.
	 * 000000,0.188162), new
	 * Trajectory.Segment(0.050000,0.224033,0.022624,0.225625,0.475000,0.500000,-0.
	 * 000000,0.213358), new
	 * Trajectory.Segment(0.050000,0.247783,0.028109,0.250000,0.500000,0.500000,0.
	 * 000000,0.240691), new
	 * Trajectory.Segment(0.050000,0.271973,0.034416,0.275000,0.500000,0.000000,-10.
	 * 000000,0.269505), new
	 * Trajectory.Segment(0.050000,0.295969,0.041426,0.300000,0.500000,0.000000,0.
	 * 000000,0.299050), new
	 * Trajectory.Segment(0.050000,0.319745,0.049149,0.325000,0.500000,0.000000,0.
	 * 000000,0.329251), new
	 * Trajectory.Segment(0.050000,0.343275,0.057594,0.350000,0.500000,0.000000,0.
	 * 000000,0.360019), new
	 * Trajectory.Segment(0.050000,0.366531,0.066765,0.375000,0.500000,0.000000,0.
	 * 000000,0.391252), new
	 * Trajectory.Segment(0.050000,0.389488,0.076661,0.400000,0.500000,0.000000,0.
	 * 000000,0.422836), new
	 * Trajectory.Segment(0.050000,0.412119,0.087280,0.425000,0.500000,0.000000,0.
	 * 000000,0.454645), new
	 * Trajectory.Segment(0.050000,0.434400,0.098615,0.450000,0.500000,0.000000,0.
	 * 000000,0.486545), new
	 * Trajectory.Segment(0.050000,0.456309,0.110655,0.475000,0.500000,0.000000,0.
	 * 000000,0.518399), new
	 * Trajectory.Segment(0.050000,0.477825,0.123384,0.500000,0.500000,0.000000,0.
	 * 000000,0.550066), new
	 * Trajectory.Segment(0.050000,0.498928,0.136785,0.525000,0.500000,0.000000,0.
	 * 000000,0.581407), new
	 * Trajectory.Segment(0.050000,0.519604,0.150837,0.550000,0.500000,0.000000,0.
	 * 000000,0.612286), new
	 * Trajectory.Segment(0.050000,0.539841,0.165514,0.575000,0.500000,0.000000,0.
	 * 000000,0.642576), new
	 * Trajectory.Segment(0.050000,0.559630,0.180790,0.600000,0.500000,0.000000,0.
	 * 000000,0.672161), new
	 * Trajectory.Segment(0.050000,0.578964,0.196638,0.625000,0.500000,0.000000,0.
	 * 000000,0.700935), new
	 * Trajectory.Segment(0.050000,0.597841,0.213027,0.650000,0.500000,0.000000,0.
	 * 000000,0.728805), new
	 * Trajectory.Segment(0.050000,0.616263,0.229927,0.675000,0.500000,0.000000,0.
	 * 000000,0.755695), new
	 * Trajectory.Segment(0.050000,0.634233,0.247306,0.700000,0.500000,0.000000,0.
	 * 000000,0.781539), new
	 * Trajectory.Segment(0.050000,0.651757,0.265135,0.725000,0.500000,0.000000,0.
	 * 000000,0.806289), new
	 * Trajectory.Segment(0.050000,0.668845,0.283383,0.750000,0.500000,0.000000,0.
	 * 000000,0.829907), new
	 * Trajectory.Segment(0.050000,0.685509,0.302019,0.775000,0.500000,0.000000,0.
	 * 000000,0.852368), new
	 * Trajectory.Segment(0.050000,0.701760,0.321015,0.800000,0.500000,0.000000,0.
	 * 000000,0.873658), new
	 * Trajectory.Segment(0.050000,0.717615,0.340344,0.825000,0.500000,0.000000,0.
	 * 000000,0.893772), new
	 * Trajectory.Segment(0.050000,0.733090,0.359979,0.850000,0.500000,0.000000,0.
	 * 000000,0.912713), new
	 * Trajectory.Segment(0.050000,0.748201,0.379894,0.875000,0.500000,0.000000,0.
	 * 000000,0.930489), new
	 * Trajectory.Segment(0.050000,0.762968,0.400066,0.900000,0.500000,0.000000,0.
	 * 000000,0.947114), new
	 * Trajectory.Segment(0.050000,0.777409,0.420473,0.925000,0.500000,0.000000,0.
	 * 000000,0.962607), new
	 * Trajectory.Segment(0.050000,0.791544,0.441094,0.950000,0.500000,0.000000,0.
	 * 000000,0.976988), new
	 * Trajectory.Segment(0.050000,0.805392,0.461908,0.975000,0.500000,0.000000,0.
	 * 000000,0.990279), new
	 * Trajectory.Segment(0.050000,0.818973,0.482897,1.000000,0.500000,0.000000,0.
	 * 000000,1.002505), new
	 * Trajectory.Segment(0.050000,0.832308,0.504043,1.025000,0.500000,0.000000,0.
	 * 000000,1.013688), new
	 * Trajectory.Segment(0.050000,0.845417,0.525331,1.050000,0.500000,0.000000,0.
	 * 000000,1.023852), new
	 * Trajectory.Segment(0.050000,0.858319,0.546744,1.075000,0.500000,0.000000,0.
	 * 000000,1.033020), new
	 * Trajectory.Segment(0.050000,0.871035,0.568269,1.100000,0.500000,0.000000,0.
	 * 000000,1.041213), new
	 * Trajectory.Segment(0.050000,0.883584,0.589891,1.125000,0.500000,0.000000,0.
	 * 000000,1.048452), new
	 * Trajectory.Segment(0.050000,0.895987,0.611597,1.150000,0.500000,0.000000,0.
	 * 000000,1.054754), new
	 * Trajectory.Segment(0.050000,0.908263,0.633376,1.175000,0.500000,0.000000,0.
	 * 000000,1.060135), new
	 * Trajectory.Segment(0.050000,0.920431,0.655214,1.200000,0.500000,0.000000,0.
	 * 000000,1.064611), new
	 * Trajectory.Segment(0.050000,0.932511,0.677102,1.225000,0.500000,0.000000,0.
	 * 000000,1.068194), new
	 * Trajectory.Segment(0.050000,0.944523,0.699027,1.250000,0.500000,0.000000,0.
	 * 000000,1.070892), new
	 * Trajectory.Segment(0.050000,0.956485,0.720980,1.275000,0.500000,0.000000,0.
	 * 000000,1.072714), new
	 * Trajectory.Segment(0.050000,0.968416,0.742949,1.300000,0.500000,0.000000,0.
	 * 000000,1.073664), new
	 * Trajectory.Segment(0.050000,0.980337,0.764924,1.325000,0.500000,0.000000,0.
	 * 000000,1.073745), new
	 * Trajectory.Segment(0.050000,0.992265,0.786895,1.350000,0.500000,0.000000,0.
	 * 000000,1.072958), new
	 * Trajectory.Segment(0.050000,1.004219,0.808851,1.375000,0.500000,0.000000,0.
	 * 000000,1.071300), new
	 * Trajectory.Segment(0.050000,1.016220,0.830783,1.400000,0.500000,0.000000,0.
	 * 000000,1.068767), new
	 * Trajectory.Segment(0.050000,1.028286,0.852678,1.425000,0.500000,0.000000,0.
	 * 000000,1.065352), new
	 * Trajectory.Segment(0.050000,1.040436,0.874527,1.450000,0.500000,0.000000,0.
	 * 000000,1.061044), new
	 * Trajectory.Segment(0.050000,1.052691,0.896318,1.475000,0.500000,0.000000,0.
	 * 000000,1.055833), new
	 * Trajectory.Segment(0.050000,1.065068,0.918038,1.500000,0.500000,0.000000,0.
	 * 000000,1.049705), new
	 * Trajectory.Segment(0.050000,1.077588,0.939677,1.525000,0.500000,0.000000,0.
	 * 000000,1.042644), new
	 * Trajectory.Segment(0.050000,1.090272,0.961221,1.550000,0.500000,0.000000,0.
	 * 000000,1.034631), new
	 * Trajectory.Segment(0.050000,1.103137,0.982656,1.575000,0.500000,0.000000,0.
	 * 000000,1.025648), new
	 * Trajectory.Segment(0.050000,1.116205,1.003969,1.600000,0.500000,0.000000,0.
	 * 000000,1.015672), new
	 * Trajectory.Segment(0.050000,1.129496,1.025143,1.625000,0.500000,0.000000,0.
	 * 000000,1.004682), new
	 * Trajectory.Segment(0.050000,1.143030,1.046163,1.650000,0.500000,0.000000,0.
	 * 000000,0.992654), new
	 * Trajectory.Segment(0.050000,1.156827,1.067011,1.675000,0.500000,0.000000,0.
	 * 000000,0.979565), new
	 * Trajectory.Segment(0.050000,1.170906,1.087669,1.700000,0.500000,0.000000,0.
	 * 000000,0.965390), new
	 * Trajectory.Segment(0.050000,1.185288,1.108118,1.725000,0.500000,0.000000,0.
	 * 000000,0.950108), new
	 * Trajectory.Segment(0.050000,1.199992,1.128336,1.750000,0.500000,0.000000,0.
	 * 000000,0.933696), new
	 * Trajectory.Segment(0.050000,1.215037,1.148301,1.775000,0.500000,0.000000,0.
	 * 000000,0.916137), new
	 * Trajectory.Segment(0.050000,1.230443,1.167990,1.800000,0.500000,0.000000,0.
	 * 000000,0.897416), new
	 * Trajectory.Segment(0.050000,1.246225,1.187379,1.825000,0.500000,0.000000,0.
	 * 000000,0.877523), new
	 * Trajectory.Segment(0.050000,1.262232,1.206557,1.850000,0.500000,0.000000,0.
	 * 000000,0.878017), new
	 * Trajectory.Segment(0.050000,1.278135,1.225847,1.875000,0.500000,0.000000,0.
	 * 000000,0.884552), new
	 * Trajectory.Segment(0.050000,1.293917,1.245236,1.900000,0.500000,0.000000,0.
	 * 000000,0.890557), new
	 * Trajectory.Segment(0.050000,1.309587,1.264715,1.925000,0.500000,0.000000,0.
	 * 000000,0.896039), new
	 * Trajectory.Segment(0.050000,1.325155,1.284276,1.950000,0.500000,0.000000,0.
	 * 000000,0.901004), new
	 * Trajectory.Segment(0.050000,1.340631,1.303910,1.975000,0.500000,0.000000,0.
	 * 000000,0.905459), new
	 * Trajectory.Segment(0.050000,1.356025,1.323609,2.000000,0.500000,0.000000,0.
	 * 000000,0.909409), new
	 * Trajectory.Segment(0.050000,1.371345,1.343364,2.025000,0.500000,0.000000,0.
	 * 000000,0.912859), new
	 * Trajectory.Segment(0.050000,1.386602,1.363169,2.050000,0.500000,0.000000,0.
	 * 000000,0.915813), new
	 * Trajectory.Segment(0.050000,1.401806,1.383015,2.075000,0.500000,0.000000,0.
	 * 000000,0.918277), new
	 * Trajectory.Segment(0.050000,1.416965,1.402894,2.100000,0.500000,0.000000,0.
	 * 000000,0.920251), new
	 * Trajectory.Segment(0.050000,1.432090,1.422800,2.125000,0.500000,0.000000,0.
	 * 000000,0.921741), new
	 * Trajectory.Segment(0.050000,1.447190,1.442724,2.150000,0.500000,0.000000,0.
	 * 000000,0.922747), new
	 * Trajectory.Segment(0.050000,1.462275,1.462660,2.175000,0.500000,0.000000,0.
	 * 000000,0.923270), new
	 * Trajectory.Segment(0.050000,1.477354,1.482601,2.200000,0.500000,0.000000,0.
	 * 000000,0.923312), new
	 * Trajectory.Segment(0.050000,1.492437,1.502538,2.225000,0.500000,0.000000,0.
	 * 000000,0.922873), new
	 * Trajectory.Segment(0.050000,1.507534,1.522465,2.250000,0.500000,0.000000,0.
	 * 000000,0.921952), new
	 * Trajectory.Segment(0.050000,1.522653,1.542375,2.275000,0.500000,0.000000,0.
	 * 000000,0.920548), new
	 * Trajectory.Segment(0.050000,1.537806,1.562260,2.300000,0.500000,0.000000,0.
	 * 000000,0.918659), new
	 * Trajectory.Segment(0.050000,1.553001,1.582112,2.325000,0.500000,0.000000,0.
	 * 000000,0.916282), new
	 * Trajectory.Segment(0.050000,1.568248,1.601924,2.350000,0.500000,0.000000,0.
	 * 000000,0.913414), new
	 * Trajectory.Segment(0.050000,1.583556,1.621689,2.375000,0.500000,0.000000,0.
	 * 000000,0.910051), new
	 * Trajectory.Segment(0.050000,1.598936,1.641398,2.400000,0.500000,0.000000,0.
	 * 000000,0.906190), new
	 * Trajectory.Segment(0.050000,1.614397,1.661044,2.425000,0.500000,0.000000,0.
	 * 000000,0.901824), new
	 * Trajectory.Segment(0.050000,1.629949,1.680619,2.450000,0.500000,0.000000,0.
	 * 000000,0.896949), new
	 * Trajectory.Segment(0.050000,1.645600,1.700113,2.475000,0.500000,0.000000,0.
	 * 000000,0.891558), new
	 * Trajectory.Segment(0.050000,1.661362,1.719518,2.500000,0.500000,0.000000,0.
	 * 000000,0.885645), new
	 * Trajectory.Segment(0.050000,1.677243,1.738826,2.525000,0.500000,0.000000,0.
	 * 000000,0.879204), new
	 * Trajectory.Segment(0.050000,1.693253,1.758027,2.550000,0.500000,0.000000,0.
	 * 000000,0.872226), new
	 * Trajectory.Segment(0.050000,1.709402,1.777111,2.575000,0.500000,0.000000,0.
	 * 000000,0.864704), new
	 * Trajectory.Segment(0.050000,1.725700,1.796068,2.600000,0.500000,0.000000,0.
	 * 000000,0.856631), new
	 * Trajectory.Segment(0.050000,1.742155,1.814889,2.625000,0.500000,0.000000,0.
	 * 000000,0.847999), new
	 * Trajectory.Segment(0.050000,1.758777,1.833563,2.650000,0.500000,0.000000,0.
	 * 000000,0.838800), new
	 * Trajectory.Segment(0.050000,1.775576,1.852078,2.675000,0.500000,0.000000,0.
	 * 000000,0.829026), new
	 * Trajectory.Segment(0.050000,1.792560,1.870423,2.700000,0.500000,0.000000,0.
	 * 000000,0.818670), new
	 * Trajectory.Segment(0.050000,1.809738,1.888585,2.725000,0.500000,0.000000,0.
	 * 000000,0.807725), new
	 * Trajectory.Segment(0.050000,1.827120,1.906554,2.750000,0.500000,0.000000,0.
	 * 000000,0.796184), new
	 * Trajectory.Segment(0.050000,1.844713,1.924316,2.775000,0.500000,0.000000,0.
	 * 000000,0.784042), new
	 * Trajectory.Segment(0.050000,1.862526,1.941857,2.800000,0.500000,0.000000,0.
	 * 000000,0.771293), new
	 * Trajectory.Segment(0.050000,1.880566,1.959164,2.825000,0.500000,0.000000,0.
	 * 000000,0.757935), new
	 * Trajectory.Segment(0.050000,1.898841,1.976223,2.850000,0.500000,0.000000,0.
	 * 000000,0.743965), new
	 * Trajectory.Segment(0.050000,1.917358,1.993020,2.875000,0.500000,0.000000,0.
	 * 000000,0.729383), new
	 * Trajectory.Segment(0.050000,1.936122,2.009539,2.900000,0.500000,0.000000,0.
	 * 000000,0.714191), new
	 * Trajectory.Segment(0.050000,1.955141,2.025765,2.925000,0.500000,0.000000,0.
	 * 000000,0.698392), new
	 * Trajectory.Segment(0.050000,1.974418,2.041683,2.950000,0.500000,0.000000,0.
	 * 000000,0.681992), new
	 * Trajectory.Segment(0.050000,1.993958,2.057277,2.975000,0.500000,0.000000,0.
	 * 000000,0.665001), new
	 * Trajectory.Segment(0.050000,2.013765,2.072531,3.000000,0.500000,0.000000,0.
	 * 000000,0.647430), new
	 * Trajectory.Segment(0.050000,2.033840,2.087428,3.025000,0.500000,0.000000,0.
	 * 000000,0.629296), new
	 * Trajectory.Segment(0.050000,2.054187,2.101954,3.050000,0.500000,0.000000,0.
	 * 000000,0.610615), new
	 * Trajectory.Segment(0.050000,2.074805,2.116092,3.075000,0.500000,0.000000,0.
	 * 000000,0.591412), new
	 * Trajectory.Segment(0.050000,2.095694,2.129826,3.100000,0.500000,0.000000,0.
	 * 000000,0.571711), new
	 * Trajectory.Segment(0.050000,2.116853,2.143140,3.125000,0.500000,0.000000,0.
	 * 000000,0.551541), new
	 * Trajectory.Segment(0.050000,2.138279,2.156021,3.150000,0.500000,0.000000,0.
	 * 000000,0.530937), new
	 * Trajectory.Segment(0.050000,2.159969,2.168453,3.175000,0.500000,0.000000,0.
	 * 000000,0.509935), new
	 * Trajectory.Segment(0.050000,2.181916,2.180422,3.200000,0.500000,0.000000,0.
	 * 000000,0.488574), new
	 * Trajectory.Segment(0.050000,2.204117,2.191917,3.225000,0.500000,0.000000,0.
	 * 000000,0.466899), new
	 * Trajectory.Segment(0.050000,2.226562,2.202924,3.250000,0.500000,0.000000,0.
	 * 000000,0.444953), new
	 * Trajectory.Segment(0.050000,2.249245,2.213434,3.275000,0.500000,0.000000,0.
	 * 000000,0.422786), new
	 * Trajectory.Segment(0.050000,2.272156,2.223437,3.300000,0.500000,0.000000,0.
	 * 000000,0.400446), new
	 * Trajectory.Segment(0.050000,2.295286,2.232923,3.325000,0.500000,0.000000,0.
	 * 000000,0.377985), new
	 * Trajectory.Segment(0.050000,2.318623,2.241887,3.350000,0.500000,0.000000,0.
	 * 000000,0.355454), new
	 * Trajectory.Segment(0.050000,2.342157,2.250322,3.375000,0.500000,0.000000,0.
	 * 000000,0.332904), new
	 * Trajectory.Segment(0.050000,2.365874,2.258225,3.400000,0.500000,0.000000,0.
	 * 000000,0.310385), new
	 * Trajectory.Segment(0.050000,2.389763,2.265593,3.425000,0.500000,0.000000,0.
	 * 000000,0.287947), new
	 * Trajectory.Segment(0.050000,2.413811,2.272424,3.450000,0.500000,0.000000,0.
	 * 000000,0.265637), new
	 * Trajectory.Segment(0.050000,2.438005,2.278719,3.475000,0.500000,0.000000,0.
	 * 000000,0.243500), new
	 * Trajectory.Segment(0.050000,2.462332,2.284480,3.500000,0.500000,0.000000,0.
	 * 000000,0.221579), new
	 * Trajectory.Segment(0.050000,2.486778,2.289709,3.525000,0.500000,0.000000,0.
	 * 000000,0.199913), new
	 * Trajectory.Segment(0.050000,2.510836,2.294321,3.549496,0.479829,-0.403422,-8.
	 * 068446,0.178966), new
	 * Trajectory.Segment(0.050000,2.533868,2.298254,3.572862,0.454829,-0.500000,-1.
	 * 931554,0.159272), new
	 * Trajectory.Segment(0.050000,2.555736,2.301560,3.594979,0.429829,-0.500000,-0.
	 * 000000,0.140913), new
	 * Trajectory.Segment(0.050000,2.576420,2.304314,3.615845,0.404829,-0.500000,0.
	 * 000000,0.123858), new
	 * Trajectory.Segment(0.050000,2.595904,2.306583,3.635461,0.379829,-0.500000,-0.
	 * 000000,0.108073), new
	 * Trajectory.Segment(0.050000,2.614177,2.308431,3.653828,0.354829,-0.500000,0.
	 * 000000,0.093519), new
	 * Trajectory.Segment(0.050000,2.631229,2.309915,3.670944,0.329829,-0.500000,0.
	 * 000000,0.080160), new
	 * Trajectory.Segment(0.050000,2.647052,2.311089,3.686811,0.304829,-0.500000,-0.
	 * 000000,0.067955), new
	 * Trajectory.Segment(0.050000,2.661640,2.312000,3.701427,0.279829,-0.500000,0.
	 * 000000,0.056867), new
	 * Trajectory.Segment(0.050000,2.674988,2.312693,3.714794,0.254829,-0.500000,-0.
	 * 000000,0.046859), new
	 * Trajectory.Segment(0.050000,2.687094,2.313206,3.726910,0.229829,-0.500000,0.
	 * 000000,0.037897), new
	 * Trajectory.Segment(0.050000,2.697954,2.313575,3.737777,0.204829,-0.500000,0.
	 * 000000,0.029949), new
	 * Trajectory.Segment(0.050000,2.707567,2.313829,3.747393,0.179829,-0.500000,-0.
	 * 000000,0.022987), new
	 * Trajectory.Segment(0.050000,2.715932,2.313996,3.755759,0.154829,-0.500000,0.
	 * 000000,0.016984), new
	 * Trajectory.Segment(0.050000,2.723047,2.314099,3.762876,0.129829,-0.500000,0.
	 * 000000,0.011919), new
	 * Trajectory.Segment(0.050000,2.728914,2.314157,3.768742,0.104829,-0.500000,0.
	 * 000000,0.007771), new
	 * Trajectory.Segment(0.050000,2.733530,2.314185,3.773359,0.079829,-0.500000,-0.
	 * 000000,0.004524), new
	 * Trajectory.Segment(0.050000,2.736896,2.314197,3.776725,0.054829,-0.500000,0.
	 * 000000,0.002166), new
	 * Trajectory.Segment(0.050000,2.739013,2.314200,3.778842,0.029829,-0.500000,-0.
	 * 000000,0.000688), new
	 * Trajectory.Segment(0.050000,2.739879,2.314200,3.779708,0.004829,-0.500000,0.
	 * 000000,0.000084), new
	 * Trajectory.Segment(0.050000,2.740000,2.314200,3.779829,0.000000,-0.096578,8.
	 * 068446,0.000000), new
	 * Trajectory.Segment(0.050000,2.740000,2.314200,3.779829,0.000000,0.000000,1.
	 * 931554,0.000000), }; // segmments for right switch Trajectory.Segment[] segs
	 * = new Trajectory.Segment[] { new
	 * Trajectory.Segment(0.050000,0.000598,0.000000,0.000625,0.025000,0.500000,10.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.002473,0.000000,0.002500,0.050000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.005598,0.000000,0.005625,0.075000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.009973,0.000000,0.010000,0.100000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.015598,0.000000,0.015625,0.125000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.022473,0.000000,0.022500,0.150000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.030598,0.000000,0.030625,0.175000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.039973,0.000000,0.040000,0.200000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.050598,0.000000,0.050625,0.225000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.062473,0.000000,0.062500,0.250000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.075598,0.000000,0.075625,0.275000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.089973,0.000000,0.090000,0.300000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.105598,0.000000,0.105625,0.325000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.122473,0.000000,0.122500,0.350000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.140598,0.000000,0.140625,0.375000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.159973,0.000000,0.160000,0.400000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.180598,0.000000,0.180625,0.425000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.202473,0.000000,0.202500,0.450000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.225598,0.000000,0.225625,0.475000,0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.249973,0.000000,0.250000,0.500000,0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.274973,0.000000,0.275000,0.500000,0.000000,-10.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.299973,0.000000,0.300000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.324973,0.000000,0.325000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.349973,0.000000,0.350000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.374973,0.000000,0.375000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.399973,0.000000,0.400000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.424973,0.000000,0.425000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.449973,0.000000,0.450000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.474973,0.000000,0.475000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.499973,0.000000,0.500000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.524973,0.000000,0.525000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.549973,0.000000,0.550000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.574973,0.000000,0.575000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.599973,0.000000,0.600000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.624973,0.000000,0.625000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.649973,0.000000,0.650000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.674973,0.000000,0.675000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.699973,0.000000,0.700000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.724973,0.000000,0.725000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.749973,0.000000,0.750000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.774973,0.000000,0.775000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.799973,0.000000,0.800000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.824973,0.000000,0.825000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.849973,0.000000,0.850000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.874973,0.000000,0.875000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.899973,0.000000,0.900000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.924973,0.000000,0.925000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.949973,0.000000,0.950000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.974973,0.000000,0.975000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,0.999973,0.000000,1.000000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.024973,0.000000,1.025000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.049973,0.000000,1.050000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.074973,0.000000,1.075000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.099973,0.000000,1.100000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.124973,0.000000,1.125000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.149973,0.000000,1.150000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.174973,0.000000,1.175000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.199973,0.000000,1.200000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.224973,0.000000,1.225000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.249973,0.000000,1.250000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.274973,0.000000,1.275000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.299973,0.000000,1.300000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.324973,0.000000,1.325000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.349973,0.000000,1.350000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.374973,0.000000,1.375000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.399973,0.000000,1.400000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.424973,0.000000,1.425000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.449973,0.000000,1.450000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.474973,0.000000,1.475000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.499973,0.000000,1.500000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.524973,0.000000,1.525000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.549973,0.000000,1.550000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.574973,0.000000,1.575000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.599973,0.000000,1.600000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.624973,0.000000,1.625000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.649973,0.000000,1.650000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.674973,0.000000,1.675000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.699973,0.000000,1.700000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.724973,0.000000,1.725000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.749973,0.000000,1.750000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.774973,0.000000,1.775000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.799973,0.000000,1.800000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.824973,0.000000,1.825000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.849973,0.000000,1.850000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.874973,0.000000,1.875000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.899973,0.000000,1.900000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.924973,0.000000,1.925000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.949973,0.000000,1.950000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.974973,0.000000,1.975000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,1.999973,0.000000,2.000000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.024973,0.000000,2.025000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.049973,0.000000,2.050000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.074973,0.000000,2.075000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.099973,0.000000,2.100000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.124973,0.000000,2.125000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.149973,0.000000,2.150000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.174973,0.000000,2.175000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.199973,0.000000,2.200000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.224973,0.000000,2.225000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.249973,0.000000,2.250000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.274973,0.000000,2.275000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.299973,0.000000,2.300000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.324973,0.000000,2.325000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.349973,0.000000,2.350000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.374973,0.000000,2.375000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.399973,0.000000,2.400000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.424973,0.000000,2.425000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.449973,0.000000,2.450000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.474973,0.000000,2.475000,0.500000,0.000000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.499723,0.000000,2.499751,0.490027,-0.199452,-3.
	 * 989040,0.000000), new
	 * Trajectory.Segment(0.050000,2.523600,0.000000,2.523627,0.465027,-0.500000,-6.
	 * 010960,0.000000), new
	 * Trajectory.Segment(0.050000,2.546226,0.000000,2.546253,0.440027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.567602,0.000000,2.567630,0.415027,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.587729,0.000000,2.587756,0.390027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.606605,0.000000,2.606633,0.365027,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.624232,0.000000,2.624259,0.340027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.640608,0.000000,2.640635,0.315027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.655734,0.000000,2.655762,0.290027,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.669611,0.000000,2.669638,0.265027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.682237,0.000000,2.682264,0.240027,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.693613,0.000000,2.693641,0.215027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.703740,0.000000,2.703767,0.190027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.712616,0.000000,2.712643,0.165027,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.720242,0.000000,2.720270,0.140027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.726619,0.000000,2.726646,0.115027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.731745,0.000000,2.731773,0.090027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.735622,0.000000,2.735649,0.065027,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.738248,0.000000,2.738275,0.040027,-0.500000,0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.739624,0.000000,2.739652,0.015027,-0.500000,-0.
	 * 000000,0.000000), new
	 * Trajectory.Segment(0.050000,2.740000,0.000000,2.740027,0.000000,-0.300548,3.
	 * 989040,0.000000), new
	 * Trajectory.Segment(0.050000,2.740000,0.000000,2.740027,0.000000,0.000000,6.
	 * 010960,0.000000), };
	 * 
	 */
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
