package org.usfirst.frc.team811.robot;

public interface Constants {

	// Port

	int FRONT_RIGHT_PORT = 3;
	int FRONT_LEFT_PORT = 1;
	int BACK_RIGHT_PORT = 4;
	int BACK_LEFT_PORT = 2;

	int DRIVE_ENCODER_PORT_LEFT_1 = 2;
	int DRIVE_ENCODER_PORT_LEFT_2 = 3;

	int DRIVE_ENCODER_PORT_RIGHT_1 = 3;
	int DRIVE_ENCODER_PORT_RIGHT_2 = 4;

	int INTAKE_TALON_PORT = 0;
	int INTAKE_LIMIT_PORT = 6;

	int SHOOTER_TALON_1_PORT = 5;
	// int SHOOTER_TALON_2_PORT = 6;
	int SHOOTER_ENCODER_PORT_1 = 0;
	int SHOOTER_ENCODER_PORT_2 = 1;

	int LIFTER_TALON_PORT = 7;
	int LIFTER_LIMIT_TOP_PORT = 8;
	int LIFTER_LIMIT_BOTTOM_PORT = 11;

	int CLIMBER_TALON_1_PORT = 1;
	int CLIMBER_TALON_2_PORT = 0;

	int SERVO_PORT = 9;

	int LIMIT_CLIMBERTOP_PORT = 9;
	int LIMIT_CLIMBERBOTTOM_PORT = 5;
	// variables

	double SPEED_SCALE = 0.85;
	double DRIVE_DISTANCE_PER_PULSE = 1 / 9.5;
	double GYRO_DIFFERENCE_VALUE = 10; // TODO

	int INTAKE_SPEED = 1; // TODO

	int SHOOTER_SPEED = 1; // TODO
	double SHOOTER_WAIT_TIME = 50; // TODO
	double SHOOTER_END_WAIT_TIME = 50; // TODO
	double SHOOTER_FULL_SPEED_RATE = 20; // TODO
	double SHOOTER_DISTANCE_PER_PULSE = 260; // TODO

	// Turret Vision Config
	double tkP = 0.01;
	double tkI = 0.00;
	double tkD = 0.4;
	double tkF = 0.00;
	double kToleranceDegrees = 1.0f;
	int framesizeX = 260;
	int framesizeY = 195;
	int framethres = 5;
	double AREA_TO_DISTANCE = 30 / 600; // TODO distance/area
	double HEIGHT_TO_DISTANCE = 30 / 600; // TODO distance/height

	boolean CAN_SHOOT = false;

	int CLIMBER_UP_POSITION = 500; // TODO
	int CLIMBER_DOWN_POSITION = 0; // TODO
	int CLIMBER_FORWARD_LIMIT = 700; // TODO
	int CLIMBER_REVERSE_LIMIT = 0; // TODO
	int CLIMBER_DIFFERENCE_VALUE = 20; // TODO

	double kP = 0.03;
	double kI = 0.00;
	double kD = 0.00;
	double kF = 0.00;

	// controls
	// driver
	int FORWARD_DRIVE_AXIS = 1;
	int TURN_DRIVE_AXIS = 4;
	int GYRO_RESET_BUTTON = 1;

	int SERVO_AXIS = 5; // triggers
	int SERVO_PRESET_BUTTON = 2;

	// operator
	int INTAKE_IN_BUTTON = 1; // a
	int INTAKE_OUT_BUTTON = 2; // b
	int INTAKE_STOP_BUTTON = 7; // back

	int CLIMBER_UP_BUTTON = 6;
	int CLIMBER_DOWN_BUTTON = 5;

	int SHOOTER_BUTTON = 3;
	int PORT_LIFT_AXIS = 1;

	int WINCH_DOWN_BUTTON = 8;

	int SHOOTER_DISTANCE_BUTTON = 4;
}
