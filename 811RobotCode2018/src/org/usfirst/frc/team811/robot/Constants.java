package org.usfirst.frc.team811.robot;

public interface Constants {

	// Port

	// Can BUS IDs
	int COMPRESSOR_PORT = 9;

	int WINCH_PORT = 1;

	int FRONT_RIGHT_PORT = 6;
	int BACK_RIGHT_PORT = 7;

	int FRONT_LEFT_PORT = 2;
	int BACK_LEFT_PORT = 3;

	int FOURBAR_LEFT_PORT = 4;
	int FOURBAR_RIGHT_PORT = 5;
	// end Can Bus Ids

	// pnuematic controller ports
	int OPEN_PORT = 0;
	int CLOSE_PORT = 1;

	// variables

	double SPEED_SCALE = 0.7;
	double ROTATE_SCALE = 0.6;
	double DRIVE_DISTANCE_PER_PULSE = 1 / 9.5;
	double GYRO_DIFFERENCE_VALUE = 10; // TODO

	int INTAKE_SPEED = 1; // TODO

	boolean CAN_SHOOT = false;

	int FOURBAR_UP_POSITION = 0; // TODO
	int FOURBAR_DOWN_POSITION = 0; // TODO
	int FOURBAR_CLIMB_POSITION = 0; // TODO
	int FOURBAR_SWITCH_POSITION = 0; // TODO

	// controls

	// driver
	int FORWARD_DRIVE_AXIS = 1;
	int TURN_DRIVE_AXIS = 4;
	int INVERSE_CONTROLS = 6; // TODO

	// operator
	int INTAKE_CLOSE_BUTTON = 6;
	int INTAKE_OPEN_BUTTON = 5;// b
	// int INTAKE_STOP_BUTTON = 7; // back

	// Four Bar
	int FOURBAR_UP_BUTTON = 3;
	int FOURBAR_DOWN_BUTTON = 1;
	int FOURBAR_SWITCH_BUTTON = 2;
	int FOURBAR_CLIMB_BUTTON = 4;
	int FOURBAR_AXIS = 1; // TODO

	int CLIMBER_UP_BUTTON = 6;
	int CLIMBER_DOWN_BUTTON = 5;   

	int SHOOTER_BUTTON = 3;
	int PORT_LIFT_AXIS = 1;

	int WINCH_DOWN_BUTTON = 8;

	int SHOOTER_DISTANCE_BUTTON = 4;
}
