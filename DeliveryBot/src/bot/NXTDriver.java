package bot;

/*
 * NXTDriver
 * Topics in Mobile Programming, Spring 2016
 * 
 */

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.MoveController;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import lejos.util.Delay;
	
public class NXTDriver {
	public static Navigator nav;
	public static DataInputStream inStream;
	public static NXTConnection btConn;
	private static ArrayList<Waypoint> intersections = new ArrayList<Waypoint>(10);
	private static ArrayList<Waypoint> offices = new ArrayList<Waypoint>(10);
	private static BufferedReader in;
	
	public static void main(String[] args) {
		//initialize Navigator
		NXTRegulatedMotor MotorA = new NXTRegulatedMotor(MotorPort.A);
		NXTRegulatedMotor MotorB = new NXTRegulatedMotor(MotorPort.B);
		nav = new Navigator(new DifferentialPilot
				(MoveController.WHEEL_SIZE_NXT1, 
				MoveController.WHEEL_SIZE_NXT1, 
				13.75, 
				MotorA, 
				MotorB, 
				true));
//		UltrasonicSensor sonic = new UltrasonicSensor(SensorPort.S1);
		System.out.println("Initializing..");
		// initialize Bluetooth connection
		try{
			btConn = Bluetooth.waitForConnection(5000, btConn.RAW);
			in = new BufferedReader(new InputStreamReader(btConn.openDataInputStream()));
			System.out.println("Connected!");
			startState();
		}catch(NullPointerException e){
			System.out.println("No Connection");
		}
		
		//add waypoints
//		nav.addWaypoint(20,0);
//		nav.addWaypoint(20, 20);
//		nav.addWaypoint(0, 20);
//		nav.addWaypoint(0,0,0);
//		Button.waitForAnyPress();
//		nav.followPath();
		Button.waitForAnyPress();
	}
	
	/*
	 * Default state
	 */
	public static void startState(){
		int command = -1;
		boolean exit = false;
		System.out.println("NXT Delivery Bot");
		System.out.println("Waiting..");
		while(!exit){
			command = -1;
			Delay.msDelay(5000);
				try {
					command = in.read();
				} catch (IOException e) {
					//e.printStackTrace();
					System.out.print("Error.");
				}
			if(command != -1){
				switch(command){
					case 0: // Test BT
						System.out.println("Connection Established!");
						btConn.close();
						break;
					case 1: // Record Waypoints
						freeMove();
						break;
					case 2: // Select Waypoints, travel
						navigate();
						break;
					case 3: //exit
						exit = true;
				}
			} else{
				//System.out.println("No Connection");
				exit = true;
			}
		}
	}
	
	/*
	 * Freely move NXT around in order to record Waypoints
	 */
	public static void freeMove(){
		int command = 0;
		boolean exit = false;
		while(!exit){
			command = 0;
			try {
				command = in.read();
			} catch (IOException e) {
				//e.printStackTrace();
				System.out.print("Error.");
			}
			
			switch(command){
			case 0: // nothing
				break;
			case 1: // forward
				nav.getMoveController().forward();
				break;
			case 2: // backward
				nav.getMoveController().backward();
				break;
			case 3: // left
				Motor.A.backward();
				Motor.B.forward();
				break;
			case 4: // right
				Motor.A.forward();
				Motor.B.backward();
				break;
			case 5: // stop
				nav.stop();
				Motor.A.stop();
				Motor.B.stop();
				break;
			case 6: // record intersection
				intersections.add(getWaypoint());
				break;
			case 7: // record office
				offices.add(getWaypoint());
				break;
			case 8: // exit
				exit = true;
			}
		}
		// when done return to start state
		startState();
	}

	/* Determine which intersections, if any, must be traveled before
	 * reaching endpoint, add to Waypoints
	 */ 
	public static void navigate(){
		
	}
	
	/*
	 * Returns a Waypoint object for the current pose
	 */
	private static Waypoint getWaypoint(){
		return new Waypoint(nav.getPoseProvider().getPose().getX(), nav.getPoseProvider().getPose().getY());
	}
}
