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
		System.out.println("Initial HeadingL "+charlie.theta());
		charlie.moveTillSense(.20);
		
		//turn robot right
		charlie.setBothSpeed(180);
		charlie.rotateRight(105); // something's wrong here I don't know what
		
//		//turn sensor
		charlie.rotateSonic(-90);
		Button.ENTER.waitForPressAndRelease();
	

//		//follow wall
		charlie.traceWall();
		Button.ENTER.waitForPressAndRelease();


// move charlie forward the length of his bod
		charlie.moveForwardDist(.15);
		
		charlie.rotateLeft((int) charlie.theta());
		System.out.println("Final Heading: " +charlie.theta());
		Button.ENTER.waitForPressAndRelease();


//		//move forward distance till goal
//		charlie.moveForwardDist(0.7);
	}

}
