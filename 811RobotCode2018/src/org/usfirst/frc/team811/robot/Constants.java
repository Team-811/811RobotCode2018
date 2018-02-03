package org.usfirst.frc.team811.robot;

import edu.wpi.first.wpilibj.Relay;


public interface Constants {
	
	//ports
	
	int FRONT_RIGHT_PORT = 3;	
	int FRONT_LEFT_PORT = 1;	
	int BACK_RIGHT_PORT = 4;	
	int BACK_LEFT_PORT = 2;		
	
	int DRIVE_ENCODER_PORT_LEFT_1 = 3;
	int DRIVE_ENCODER_PORT_LEFT_2 = 4;
	
	int DRIVE_ENCODER_PORT_RIGHT_1 = 8;
	int DRIVE_ENCODER_PORT_RIGHT_2 = 9;
	
	int ULTRA_PORT = 0; //TODO
	
	//Drive
	double SPEED_SCALE = 0.85;
	double DRIVE_DISTANCE_PER_PULSE = 1/9.5;	
	double GYRO_DIFFERENCE_VALUE = 10; //TODO

	
	
	//controls
	//driver
	int FORWARD_DRIVE_AXIS = 1;	
	int TURN_DRIVE_AXIS = 4; 	
	int GYRO_RESET_BUTTON = 1;
	
	int SERVO_AXIS = 5;	//triggers
	int SERVO_PRESET_BUTTON = 2;
	
	
}
