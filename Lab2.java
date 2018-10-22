package lab2;

public class Lab2 {

	public static void main(String[] args) {
		Charlie charlie= new Charlie();

		charlie.syncMotors();
		
		//move towards wall
		charlie.moveTillSense(.20);
		
		//turn sensor
		charlie.rotateSonic(90, true );
		
		//turn robot right
		charlie.rotateRight(90);
		
		//follow wall
		charlie.traceWall();
		
		//turn robot left
		charlie.rotateLeft(90);
		
		//move forward distance till goal
		charlie.moveForwardDist(0.7);
	}

}
