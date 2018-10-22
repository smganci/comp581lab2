package lab2;

import java.util.Arrays;

import lejos.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
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
	private double radiusL;
	private double radiusR;

	//sensor modes
	private SensorMode touchL;
	private SensorMode touchR;
	private SensorMode sonic;
	
	public Charlie() {
		this.motorL= new EV3LargeRegulatedMotor(MotorPort.B);
		this.motorR= new EV3LargeRegulatedMotor(MotorPort.C);
		this.motorM= new EV3MediumRegulatedMotor(MotorPort.A);
		this.touchSensorL=new EV3TouchSensor(SensorPort.S1);
		this.touchSensorR=new EV3TouchSensor(SensorPort.S4);
		this.sonicSensor=new EV3UltrasonicSensor(SensorPort.S3);
		
		this.touchL=this.touchSensorL.getTouchMode();
		this.touchR = this.touchSensorR.getTouchMode();
		this.sonic= (SensorMode) this.sonicSensor.getDistanceMode();
		this.radiusL=.028;
		this.radiusR=.028;
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
		//w=(ul+ur)/2
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
		float sonic=sonicSense();
		
		//basically we want to stop once we sense that the wall has ended
		//may need to alter while statement
		while(sonic <.5) {
			
			//if the sonic is too close, need to adjust to move toward a point further away
			if(sonic<.1) {
				//move away
			}
			
			
			if(sonic>.2) {
				//move closer
			}
			
			
			sonic=sonicSense();
			if(sonic >.4) {
				sonic=sonicSense();
				//
				float amt=0;
				if(sonic>amt) {
					break;
				}
			}
		}
	}
	
	/*Name: rotateSonic
	 * in: degrees
	 * out: nothing
	 * description: 
	 * should rotate sonic sensor to certain angle relative to robot
	 * 
	 * */
	public void rotateSonic(int degrees) {
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
		//need to set velocities to be opposites
		//need to turn for a specific ammount of time
		//need to stop
		float av=180;////////////////////need to decide on a reasonable speed
		
		//set speed
		this.setBothSpeed(av);
		
		//move right forward and left backward to create a spin
		this.stopSync();
		this.motorL.forward();
//		this.motorL.backward();
		
		
//		long delay= (long) (degrees/av)*1000; // need to set delay time
		long delay =500;
		Delay.msDelay(delay);
		
		
		//stop both motors
		this.stopBothInstant();
		
	}
	
	/*Name: rotateLeft
	 * in: degrees to rotate
	 * out: nothing
	 * description: should turn the robot in place towards goal
	 * */
	
	public void rotateLeft(int degrees) {
		//need to set velocities to be opposites
		//need to turn for a specific ammount of time
		//need to stop
	}
	
//	/*Name: moveTimeSpin
//	 * in: omega in degrees, theta in degrees
//	 * out: seconds to move
//	 * */
//	
//	public long moveTimeSpin(long omega, long theta) {
//		long time= theta/omeg;
//	}
//	
	
	
	
}
