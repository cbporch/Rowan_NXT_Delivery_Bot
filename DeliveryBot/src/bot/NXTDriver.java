package bot;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;

import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.MoveController;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.util.Delay;

public class NXTDriver {
	public static Navigator nav;
	public static NXTConnection btConn;
	private static ArrayList<Waypoint> intersections = new ArrayList<Waypoint>(10);
	private static ArrayList<Waypoint> offices = new ArrayList<Waypoint>(10);
	private static DataInputStream in;
	private static DataOutputStream out;
	private static NXTRegulatedMotor MotorA;
	private static NXTRegulatedMotor MotorB;

	public static void main(String[] args) {
		// initialize Navigator
		MotorA = new NXTRegulatedMotor(MotorPort.A);
		MotorB = new NXTRegulatedMotor(MotorPort.B);
		nav = new Navigator(new DifferentialPilot(MoveController.WHEEL_SIZE_NXT1, MoveController.WHEEL_SIZE_NXT1, 13.75,
				MotorA, MotorB, true));

		Button.ESCAPE.addButtonListener(new ButtonListener() {
			public void buttonPressed(Button b) {
				System.exit(0);
			}

			public void buttonReleased(Button b) {
				System.exit(0);
			}
		});

		// UltrasonicSensor sonic = new UltrasonicSensor(SensorPort.S1);
		System.out.println("\n\nInitializing..");
		
		// initialize Bluetooth connection
		openConn();
		selectState();
		closeConn();
		
		//insert code here to attempt to reconnect to bluetooth?
		
		System.out.println("Press any button to exit.");
		Button.waitForAnyPress();
		
	}
	
	public static void openConn(){
		try {
			btConn = Bluetooth.waitForConnection(60000, btConn.RAW);
			in = new DataInputStream(btConn.openDataInputStream());
			out = new DataOutputStream(btConn.openOutputStream());
			LCD.clear();
			System.out.println("Connected to the best phone ever!");
			Sound.beepSequenceUp();
		} catch (NullPointerException e) {
			System.out.println("No Connection");
		}
	}

	/*
	 * Freely move NXT around in order to record Waypoints
	 */
	public static void selectState() {
		int command = 0;
		boolean exit = false;
		while (!exit) {
			command = readBTInput();
			System.out.println(command);			
			switch (command) {
				case 0: // stop forward or backward movement
					MotorA.stop(true);
					MotorB.stop(true);
					break;
				case 1: // forward
					MotorA.forward();
					MotorB.forward();
					break;
				case 2: // backward
					MotorA.backward();
					MotorB.backward();
					break;
				case 3: // left
					MotorA.forward();
					MotorB.backward();
					break;
				case 4: // right
					MotorA.backward();
					MotorB.forward();
					break;
				case 5: // send current x and y
					sendCoordinatesBT();
					break;
				case 6: // receive Coordinates
					command = receiveCoordinates();
					if(command == -1){
						exit = true;
					}
					break;
				case -1:// connection  lost
					exit = true;
					break;
				default:
					break;
			}
		}
	}

	/*
	 * Receive Coordinates from Bluetooth connection, add them to a path,
	 * then follow it
	 */
	private static int receiveCoordinates() {
		int command = readBTInput();
			while(command != -2 || command!= -1){
				float x = readBTInput(), 
						y = readBTInput(), 
						heading = readBTInput();
				nav.addWaypoint(x, y, heading);
			}
			if (command == -1)
				return command;
			nav.followPath();
			while(nav.isMoving()){
				//check ultrasonic sensor
				
			}
			// return to start
			return 0;
	}
	

	/*
	 * Returns a Waypoint object for the current pose
	 */
	private static Waypoint getWaypoint() {
		return new Waypoint(nav.getPoseProvider().getPose().getX(), nav.getPoseProvider().getPose().getY());
	}

	private static void bluetoothTest() {
		boolean exit = false;
		int command = 1;
		while (!exit) {
			try {
				command = readBTInput();
				if (command == 0 || command == -1) {
					exit = true;
					closeConn();
					System.out.println("Goodbye!");
				} else {
					System.out.println(command);
					Delay.msDelay(500);
				}

			} catch (Exception e) {
				// no command read
			}
		}
	}

	private static int readBTInput() {
		try {
			// Delay.msDelay(60);
			return in.read();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.print("Read Error");
			return 0;
		}
	}

	private static void sendCoordinatesBT() {
		Delay.msDelay(500);
		Pose p = nav.getPoseProvider().getPose();
		float x = p.getX(), y = p.getY(), heading = p.getHeading();
		try {
			String output = Float.toString(x) +"\n"+Float.toString(y)+"\n"+Float.toString(heading)+"\n";
			System.out.println("("+x+","+y+","+heading+")");
			out.writeBytes(output);
			out.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			//e.printStackTrace();
		}
	}
	
	private static void closeConn() {
		try {
				in.close();
				out.close();
				btConn.close();
				Sound.buzz();
		} catch (IOException e) {
		}
	}

	private static void test() {
		nav.getMoveController().forward();
		Delay.msDelay(2000);
		nav.stop();
		Delay.msDelay(2000);
		nav.getMoveController().backward();
		Delay.msDelay(2000);
		nav.stop();
		Delay.msDelay(2000);
		Motor.A.backward();
		Motor.B.forward();
		Delay.msDelay(2000);
		Motor.A.stop(true);
		Motor.B.stop(true);
		Delay.msDelay(2000);
		Motor.B.backward();
		Motor.A.forward();
		Delay.msDelay(2000);
		Motor.A.stop(true);
		Motor.B.stop(true);
		Delay.msDelay(2000);
		System.exit(0);

	}
}
