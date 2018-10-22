package lab2;

import lejos.hardware.Button;

public class Lab2 {

	public static void main(String[] args) {
		Charlie charlie= new Charlie();

		charlie.syncMotors();
		
		System.out.println("Objective 1");
		System.out.println("Press Button to Start");
		Button.ENTER.waitForPressAndRelease();
		//move towards wall
//		charlie.moveTillSense(.20);
//		
		//turn robot right
		System.out.println("about to rotate robot right");
		charlie.rotateRight(90);
		System.out.println("test");
		Button.ENTER.waitForPressAndRelease();
		
		
//		System.out.println("about to turn sensor");
//		//turn sensor
//		charlie.rotateSonic(-90 );
			

//		//follow wall
//		charlie.traceWall();
//		
//		//turn robot left
//		charlie.rotateLeft(90);
//		
//		//move forward distance till goal
//		charlie.moveForwardDist(0.7);
	}

}
