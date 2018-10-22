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
		
		//follow wall
		
		//turn robot left
		
		//move forward distance till goal
	}

}
