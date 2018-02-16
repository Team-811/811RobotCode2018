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

	private double MAX_SPEED = 0.4;

	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system. Note that the */
	/* SmartDashboard in Test mode has support for helping you tune */
	/* controllers by displaying a form where you can enter new P, I, */
	/* and D constants and test the mechanism. */

	private double rotateKP = 0.003;
	private double rotateKI = 0.00;
	private double rotateKD = 0.001;
	private double rotateKTolerance = 100.0;

	EncoderFollower leftFollower;
	EncoderFollower rightFollower;
	Trajectory trajectory;
	TankModifier modifier;

	double l;
	double r;

	double max_velocity = 0.5;
	double max_acceleration = 0.5;
	double max_jerk = 60.0;
	double wheel_diameter = 0.15;
	double wheel_base_distance = 0.3759;
	int encoder_rotation = 1100;
	double kD = 0.0;
	double kP = 1;
	double acceleration_gain = 0.0;
	// TODO
	double absolute_max_velocity = 0.78;
	double gyro_correction_power = 0.8;

	int leftEncoderStartingPosition;
	int rightEncoderStartingPosition;

	int occuranceNumber = 0;
	int occuranceTolerance = 10;

	public MotionProfile() {
		rotateController = new PIDController(rotateKP, rotateKI, rotateKP, this, this);
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

	public void pidWrite(double output) {
		// SmartDashboard.putNumber("strafe pid output", output);

		// Take the output of the PID loop and add the offset to hold position
		double command = output;

		// SmartDashboard.putNumber("strafe error", fourBarController.getError());
		// if (isParking && (output < 0.001)) {
		// // if parking and the output is 0 - the arm is all the way down
		// // and the motor command 0
		// command = 0.0;
		// }
		driveTrain.arcadeDrive(0, command);
	}

	public void configureFollower() {

		// driveLeft.setSelectedSensorPosition(0, 0, 1);
		// driveRight.setSelectedSensorPosition(0, 0, 1);
		ahrs.reset();

		leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
		rightFollower = new EncoderFollower(modifier.getRightTrajectory());

		leftEncoderStartingPosition = frontleft.getSelectedSensorPosition(0);
		rightEncoderStartingPosition = frontright.getSelectedSensorPosition(0);
		leftFollower.configureEncoder(leftEncoderStartingPosition, encoder_rotation, wheel_diameter);
		rightFollower.configureEncoder(rightEncoderStartingPosition, encoder_rotation, wheel_diameter);
		leftFollower.configurePIDVA(rotateKP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);
		rightFollower.configurePIDVA(rotateKP, 0.0, kD, 1 / absolute_max_velocity, acceleration_gain);

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

	public void generateDriveStraightTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0),
				// new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0
				// radians

				// new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		modifier = new TankModifier(trajectory).modify(wheel_base_distance);

		/*
		 * for (int i = 0; i < trajectory.length(); i++) { Trajectory.Segment seg =
		 * trajectory.get(i);
		 * 
		 * System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y,
		 * seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		 * 
		 * }
		 */

	}

	public void generateLeftSwitchTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0),
				// new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0
				// radians

				// new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		modifier = new TankModifier(trajectory).modify(wheel_base_distance);

		/*
		 * for (int i = 0; i < trajectory.length(); i++) { Trajectory.Segment seg =
		 * trajectory.get(i);
		 * 
		 * System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y,
		 * seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		 * 
		 * }
		 */

	}

	public void generateRightSwitchTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0),
				// new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0
				// radians

				// new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		modifier = new TankModifier(trajectory).modify(wheel_base_distance);

		/*
		 * for (int i = 0; i < trajectory.length(); i++) { Trajectory.Segment seg =
		 * trajectory.get(i);
		 * 
		 * System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y,
		 * seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		 * 
		 * }
		 */

	}

	public void generateRightSwitchToCubePickupTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0),
				// new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0
				// radians

				// new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		modifier = new TankModifier(trajectory).modify(wheel_base_distance);

		/*
		 * for (int i = 0; i < trajectory.length(); i++) { Trajectory.Segment seg =
		 * trajectory.get(i);
		 * 
		 * System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y,
		 * seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		 * 
		 * }
		 */

	}

	public void generateLeftSwitchToCubePickupTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0),
				// new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0
				// radians

				// new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		modifier = new TankModifier(trajectory).modify(wheel_base_distance);

		/*
		 * for (int i = 0; i < trajectory.length(); i++) { Trajectory.Segment seg =
		 * trajectory.get(i);
		 * 
		 * System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y,
		 * seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		 * 
		 * }
		 */

	}

	public void generateCubePickupTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0),
				// new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0
				// radians

				// new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		modifier = new TankModifier(trajectory).modify(wheel_base_distance);

		/*
		 * for (int i = 0; i < trajectory.length(); i++) { Trajectory.Segment seg =
		 * trajectory.get(i);
		 * 
		 * System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y,
		 * seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		 * 
		 * }
		 */

	}

	public void generateScaleTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0),
				// new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0
				// radians

				// new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		modifier = new TankModifier(trajectory).modify(wheel_base_distance);

		/*
		 * for (int i = 0; i < trajectory.length(); i++) { Trajectory.Segment seg =
		 * trajectory.get(i);
		 * 
		 * System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y,
		 * seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		 * 
		 * }
		 */

	}

	public void generateApproachTrajectory() {

		Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0),
				// new Waypoint(1.8288, 4.2672, 0), // Waypoint @ x=-2, y=-2, exit angle=0
				// radians

				// new Waypoint(2,-2,Pathfinder.d2r(-90))
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		trajectory = Pathfinder.generate(points, config);
		modifier = new TankModifier(trajectory).modify(wheel_base_distance);

		/*
		 * for (int i = 0; i < trajectory.length(); i++) { Trajectory.Segment seg =
		 * trajectory.get(i);
		 * 
		 * System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", seg.dt, seg.x, seg.y,
		 * seg.position, seg.velocity, seg.acceleration, seg.jerk, seg.heading);
		 * 
		 * }
		 */

	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
