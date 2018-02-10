package org.usfirst.frc.team811.robot;

public interface Constants {

	// Port

	int FRONT_RIGHT_PORT = 3;
	int FRONT_LEFT_PORT = 1;
	int BACK_RIGHT_PORT = 4;
	int BACK_LEFT_PORT = 2;

	int FOURBAR_LEFT_PORT = 1; // TODO
	int FOURBAR_RIGHT_PORT = 0;

	int COMPRESSOR_PORT = 0; // TODO
	int OPEN_PORT = 0;
	int CLOSE_PORT = 1;

	// variables

	double SPEED_SCALE = 0.85;
	double DRIVE_DISTANCE_PER_PULSE = 1 / 9.5;
	double GYRO_DIFFERENCE_VALUE = 10; // TODO

	int INTAKE_SPEED = 1; // TODO

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
	int INVERSE_CONTROLS = 6; // TODO

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
