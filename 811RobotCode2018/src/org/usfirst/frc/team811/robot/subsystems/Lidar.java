package org.usfirst.frc.team811.robot.subsystems;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SensorBase;
 
public class Lidar extends Subsystem{
	private I2C i2c;
	
	// declaring a variable as volatile results in atomic reads 
	private volatile int distance;
	
	private java.util.Timer updater;
	private LIDARUpdater task;
	
	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	public Lidar(Port port) {
		i2c = new I2C(Port.kMXP, LIDAR_ADDR);
		
		distance = 0;
 
		task = new LIDARUpdater();
		updater = new java.util.Timer();
	}
	
	// Distance in cm
	public int getDistance() {
		// needs to be thread safe.  The variable distance is written in a different thread
		// than it is read.
		
		// You want to ensure that while the two bytes of the value are being
		// written, they are not also being read
		
		return distance;
	}
 
	public void start() {
		start(20);  // 20 ms interval - 50hz
	}
	
	// Start polling for period in milliseconds
	public void start(int period) {
		updater.scheduleAtFixedRate(task, 0, period);
	}
	
	public void stop() {
		updater.cancel();
	}
	
	// Update distance variable
	public void update() {
		i2c.write(LIDAR_CONFIG_REGISTER, 0x04); // Initiate measurement
		Timer.delay(0.04); // Delay for measurement to be taken
		
		byte[] buffer = new byte[2];
		i2c.read(LIDAR_DISTANCE_REGISTER, 2, buffer); // Read in measurement
		
		distance =  (int) Integer.toUnsignedLong(buffer[0] << 8) + Byte.toUnsignedInt(buffer[1]);
		Timer.delay(0.01); // Delay to prevent over polling
	}
	
	// Timer task to keep distance updated
	private class LIDARUpdater extends TimerTask {
		public void run() {
			while(true) {
				update();
//				if(getDistance() < 90 && getDistance() > 84){
//					SmartDashboard.putBoolean("Correct distance from human feeder", true);
//				}
//				else{
//					SmartDashboard.putBoolean("Correct distance from human feeder", false);
//				}
//				
//				if(getDistance() < 80 && getDistance() > 70){
//					SmartDashboard.putBoolean("Correct distance to stacks", true);
//				}
//				else{
//					SmartDashboard.putBoolean("Correct distance to stacks", false);
//				}
//				SmartDashboard.putNumber("LIDAR distance Inches", (getDistance() / 2.54));
//				try {
//					Thread.sleep(10);
//				} catch (InterruptedException e) {
//					e.printStackTrace();
//				}
			}
		}
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}
}