package lab2;

import java.util.Arrays;

import lejos.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class Charlie {
	//components
	private EV3LargeRegulatedMotor motorL;
	private EV3LargeRegulatedMotor motorR;
	private EV3MediumRegulatedMotor motorM;
	private EV3TouchSensor touchSensorL;
	private EV3TouchSensor touchSensorR;
	private EV3UltrasonicSensor sonicSensor;
	private EV3GyroSensor gyroSensor;
	private double radiusL;
	private double radiusR;
	private double L;
	private double heading;

	//sensor modes
	private SensorMode touchL;
	private SensorMode touchR;
	private SensorMode sonic;
	private SensorMode gyro;
	
	public Charlie() {
		this.motorL= new EV3LargeRegulatedMotor(MotorPort.B);
		this.motorR= new EV3LargeRegulatedMotor(MotorPort.C);
		this.motorM= new EV3MediumRegulatedMotor(MotorPort.A);
		this.touchSensorL=new EV3TouchSensor(SensorPort.S1);
		this.touchSensorR=new EV3TouchSensor(SensorPort.S4);
		this.sonicSensor=new EV3UltrasonicSensor(SensorPort.S3);
		this.gyroSensor = new EV3GyroSensor(SensorPort.S2);
		
		this.touchL=this.touchSensorL.getTouchMode();
		this.touchR = this.touchSensorR.getTouchMode();
		this.sonic= (SensorMode) this.sonicSensor.getDistanceMode();
		this.gyro = (SensorMode) this.gyroSensor.getAngleMode();
		this.radiusL=.028;
		this.radiusR=.028;
		this.L=.12;
		this.heading = 0;
	}
	
	/*Name: setLeftSpeed
	 * in: angular velocity in degrees (float s) 
	 * out: nothing 
	 * description: sets left motor's speed to that angular velocity 
	 * */
	public void setLeftSpeed(float s) {
		this.motorL.setSpeed(s);
	}
	
	
	/*Name: setRighttSpeed
	 * in: angular velocity in degrees (float s) 
	 * out: nothing 
	 * description: sets right motor's speed to that angular velocity 
	 * */
	public void setRightSpeed(float s) {
		this.motorR.setSpeed(s);
	}
	
	/*Name: setBothSpeed
	 * in: angular velocity in degrees (float s) 
	 * out: nothing 
	 * description: sets both motors speed to s
	 * */
	public void setBothSpeed(float s) {
		this.setLeftSpeed(s);
		this.setRightSpeed(s);
	}
	
	/*Name: moveForwardBoth
	 * in: nothing 
	 * out: nothing 
	 * description: moves both motors forward
	 * */
	public void moveForwardBoth() {
		this.motorL.forward();
		this.motorR.forward();
	}
	
	/*Name: moveBackwardBoth
	 * in: nothing 
	 * out: nothing 
	 * description: moves both motors backward
	 * */
	public void moveBackwardBoth() {
		this.motorL.backward();
		this.motorR.backward();
	}
	
	/*Name: stopBothInstant
	 * in: nothing 
	 * out: nothing 
	 * description: stops both instantly
	 * */
	public void stopBothInstant() {
		this.motorL.stop(true);
		this.motorR.stop(true);
	}
	
	/*Name: moveTillSense
	 * in: distance in meters (dist) 
	 * out: nothing 
	 * description: moves robot forward till sonic sensor senses an obstacle a certain distance away
	 * */
	public void moveTillSense(double d) {
		float[] sample_sonic = new float[this.sonic.sampleSize()];
		this.sonic.fetchSample(sample_sonic, 0);
		while(sample_sonic[0] > d) {
			this.moveForwardBoth();
			sonic.fetchSample(sample_sonic, 0);
		}
		this.stopBothInstant();
	}
	
	
	/*Name: moveTillTouch
	 * in: nothing 
	 * out: nothing 
	 * description: moves robot forward till touch sensor encounters an obstacle
	 **/
	public void moveTillTouch() {
		float[] sample_touchL = new float[touchL.sampleSize()];
		float[] sample_touchR = new float[touchR.sampleSize()];

	    while(sample_touchL[0] == 0 && sample_touchR[0]==0) {
		      this.moveForwardBoth();
		      touchL.fetchSample(sample_touchL,0);
		      touchR.fetchSample(sample_touchR,0);
		}
		this.stopBothInstant();		
	}
	

	/*Name: syncMotors
	 * in: nothing
	 * out: nothing 
	 * description: synchronizes left and right motors
	 * */
	public void syncMotors() {
		this.motorL.synchronizeWith(new EV3LargeRegulatedMotor[] {this.motorR });
	}
	
	public void stopSync() {
		this.motorL.endSynchronization();
	}
	
	/*Name: moveForwardTime
	 * in: time to move in seconds
	 * out: nothing 
	 * description: makes robot move forward for a certain amount of time
	 * */
	public void moveForwardTime(long sec) {
		this.syncMotors();
		this.moveForwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
	}
	
	/*Name: moveBackwardTime
	 * in: time to move in seconds
	 * out: nothing 
	 * description: makes robot move Backward for a certain amount of time
	 * */
	public void moveBackwardTime(long sec) {
		this.syncMotors();
		this.moveBackwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
	}
	///////////////////////////////////need to alter move time to account for diff radius
	/*Name: moveTime
	 * in: time to move in seconds
	 * out: nothing 
	 * description: makes robot move Backward for a certain amount of time
	 * */	
	private long moveTime(float angularv, double d) {
		return (long) (d / (angularv * (Math.PI / 180) *(this.radiusL+this.radiusR)/2.0) * 1000);
	}
	
	/*Name: moveForwardDist
	 * in: distance in meters 
	 * out: nothing 
	 * description: moves robot forward a certain distance
	 **/
	public void moveForwardDist(double d) {
		//w=(ul+ur)/2
		float w= (this.motorL.getSpeed()+this.motorR.getSpeed())/2;
		long sec =this.moveTime(w, d);
		this.moveForwardTime(sec);
	}

	/*Name: moveBackwardDist
	 * in: distance in meters 
	 * out: nothing 
	 * description: moves robot backward a certain distance
	 **/
	public void moveBackwardDist(double d) {
		//w=(ul+ur)/2 => I don't think that's true? v=(vl+vr)/2
		float w= (this.motorL.getSpeed()+this.motorR.getSpeed())/2;
		long sec =this.moveTime(w, d);
		this.moveBackwardTime(sec);
	}

	/*Name: beep
	 * in: nothing
	 * out: nothing 
	 * description: makes robot beep
	 * */
	public void beep() {
		Sound.beep();
	}
	
	/*Name: buttonWait
	 * in: nothing
	 * out: nothing 
	 * description: waits for button press to continue
	 * */
	public void buttonWait() {
		Button.ENTER.waitForPressAndRelease();
	}
	
	/////////////NEW
	
	/*Name: traceWall
	 * in: nothing
	 * out: nothing
	 * description: traceWall should continually read from ultrasonic sensor
	 * idea: 
	 * constantly read and each reading adjust wheel speeds such that they are moving toward a point 15 cm away from the wall
	 * maybe have another method move toward that moves toward some designated point and adjusts
	 * 
	 * 
	 * */
	
	public void traceWall() {
		float sonic = sonicSense();
		
		float bs = 270;
		this.setBothSpeed(bs);
		
		this.syncMotors();
		this.moveForwardBoth();
		
		//basically we want to stop once we sense that the wall has ended
		//may need to alter while statement
		while(sonic < .5) {
			
			//if the sonic is too close, need to adjust to move toward a point further away
			if (sonic >= Float.POSITIVE_INFINITY) {
				sonic=sonicSense();
			}
			if(sonic<.1) {
				//move away
				float ls = 360;
				this.setLeftSpeed(ls);
				this.setRightSpeed(bs);
				long time = this.timeToRotate(bs, ls, 45);
				long startTime = System.currentTimeMillis(); //fetch starting time
				while((System.currentTimeMillis()-startTime)<time)
				{
					float newSonic= sonicSense();
					if (newSonic > .15) {
						break;
					}
				}
//				Delay.msDelay(time);
				this.setLeftSpeed(bs);
//				this.heading += 45;
				System.out.println("Else-if 1");
			}
			
			if(sonic>.2) {
				//move closer
				float rs = 360;
				this.setRightSpeed(rs);
				this.setLeftSpeed(bs);
				long time = this.timeToRotate(rs, bs, 45);
				long startTime = System.currentTimeMillis(); //fetch starting time
				while((System.currentTimeMillis()-startTime)<time)
				{
					float newSonic= sonicSense();
					if (newSonic < .15) {
						break;
					}
				}				this.setRightSpeed(bs);
//				this.heading -= 45;
				System.out.println("Else-if 2");
			}
			
			sonic=sonicSense();
			if(sonic >.3) {
				sonic=sonicSense();
				//
				double amt= 0.5;
				if(sonic>amt) {
					System.out.println("The current sensed value is: "+ sonic);
					break;
				}
			}
			System.out.println("End loop");

		}
		this.stopBothInstant();
	}
	
	/*Name: rotateSonic
	 * in: degrees
	 * out: nothing
	 * description: 
	 * should rotate sonic sensor to certain angle relative to robot
	 * 
	 * */
	public void rotateSonic(int degrees) {
		this.motorM.setSpeed(90);
		this.motorM.rotate(degrees);
	}
	
	/*Name: sonicSense
	 * in: nothing
	 * out: double number representing the median of the distance sensed from the sonic sensor
	 * description: senses 3 times and returns the median
	 * */
	
	public float sonicSense() {
		float [] dists= new float[3];
		float[] sample_sonic = new float[sonic.sampleSize()];
		sonic.fetchSample(sample_sonic, 0);
		dists[0]= sample_sonic[0];
		sonic.fetchSample(sample_sonic, 0);
		dists[1]= sample_sonic[0];
		sonic.fetchSample(sample_sonic, 0);
		dists[2]= sample_sonic[0];
		Arrays.sort(dists);
		return dists[1];
	}
	
	/*Name: moveToward
	 * in: 
	 * 	x: translation over x axis, 
	 * 	y: translation over y axis
	 * 	stop: boolean (stop once point is reached or keep moving in that direction)
	 * out: none
	 * description:
	 * */
	
	public void moveToward(double x, double y) {
		
	}
	
	
	
	/*Name: rotateRight
	 * in: degrees to rotate
	 * out: nothing
	 * description: should turn the robot in place towards goal
	 * */
	
	public void rotateRight(long degrees) {
		//move right forward and left backward to create a spin
		this.stopSync();
		this.motorL.forward();
//		this.motorR.backward();
		
		
//		long delay= (long) (degrees/av)*1000; // need to set delay time
		this.setLeftSpeed(180);
		long delay = this.timeToRotate(0 , 180, degrees);
		Delay.msDelay(delay);
		
		//stop both motors
		this.stopBothInstant();
		this.heading += degrees;
		
	}
	
	/*Name: rotateLeft
	 * in: degrees to rotate
	 * out: nothing
	 * description: should turn the robot in place towards goal
	 * */
	
	public void rotateLeft(int degrees) {
		
		//move right forward and left backward to create a spin
		this.stopSync();
		this.motorR.forward();
		
		this.setRightSpeed(180);
		long delay = this.timeToRotate(180, 0, degrees);
		Delay.msDelay(delay);
				
		//stop both motors
		this.stopBothInstant();
		this.heading -= degrees;
	}
	
	public float theta() {
		float[] sample_gyro = new float[sonic.sampleSize()];
		this.gyro.fetchSample(sample_gyro, 0);
		return sample_gyro[0];
	}
	
//	/*Name: timeToRotate
//	 * in: ul, ur, theta
//	 * out: seconds to move
//	 * */
//	
	public long timeToRotate(double ur, double ul, double theta) {
		double vr = ur * Math.PI/180 * this.radiusR;
		double vl = ul * Math.PI/180 * this.radiusL;
		double omega = (vl-vr)/this.L;
		double time = (theta * Math.PI/180)/omega;
		return (long) time*1000;
	}
	
	
	
}
