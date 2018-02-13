package org.usfirst.frc.team811.robot.subsystems;

import org.usfirst.frc.team811.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

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
public class MotionProfile extends Subsystem {

	WPI_TalonSRX frontright = RobotMap.drivefrontright;
	WPI_TalonSRX backright = RobotMap.drivebackright;
	SpeedControllerGroup driveRight = RobotMap.driveRight;
	WPI_TalonSRX frontleft = RobotMap.drivefrontleft;
	WPI_TalonSRX backleft = RobotMap.drivebackleft;
	SpeedControllerGroup driveLeft = RobotMap.driveLeft;
	DifferentialDrive driveTrain = RobotMap.driveTrain;
	AHRS ahrs = RobotMap.ahrs;

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

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

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

	}

	public void reset() {
		driveTrain.tankDrive(0, 0);
	}

	public Boolean isFinished() {
		if (l == 0 && r == 0) {

			occuranceNumber++;

			if (occuranceNumber >= occuranceTolerance) {
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
