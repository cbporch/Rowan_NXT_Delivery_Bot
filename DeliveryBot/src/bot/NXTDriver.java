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
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
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
	private static DataInputStream in;
	private static NXTRegulatedMotor MotorA;
	private static NXTRegulatedMotor MotorB;
	
	public static void main(String[] args) {
		//initialize Navigator
		MotorA = new NXTRegulatedMotor(MotorPort.A);
		MotorB = new NXTRegulatedMotor(MotorPort.B);
		nav = new Navigator(new DifferentialPilot
				(MoveController.WHEEL_SIZE_NXT1, 
				MoveController.WHEEL_SIZE_NXT1, 
				13.75, 
				MotorA, 
				MotorB, 
				true));
		
		Button.ESCAPE.addButtonListener(new ButtonListener() {
			   public void buttonPressed(Button b) {
				   System.exit(0);
			   }
			   public void buttonReleased(Button b) {
				   System.exit(0);
			   }
			});
		
//		UltrasonicSensor sonic = new UltrasonicSensor(SensorPort.S1);
		System.out.println("Initializing..");
		// initialize Bluetooth connection
		try{
			btConn = Bluetooth.waitForConnection(30000, btConn.RAW);
		//	in = new BufferedReader(new InputStreamReader(btConn.openInputStream()));
			in = new DataInputStream(btConn.openDataInputStream());
			LCD.clear();
			System.out.println("Connected!");
			//startState();
			freeMove();
			
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
		System.out.println("Press any button to exit.");
		Button.waitForAnyPress();
		closeConn();
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
			Delay.msDelay(1000);
			command = readBTInput();
			if(command != -1){
				switch(command){
					case 0: // Test BT
						bluetoothTest();
						break;
					case 1: // Record Waypoints
						freeMove();
						break;
					case 2: // Select Waypoints, travel
						navigate();
						break;
					case 3: //exit
						exit = true;
						System.out.println("Goodbye!");
						break;
				}
			} else {
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
			//LCD.clear();
			command = readBTInput();
			System.out.println(command);
			switch(command){
			case 0: // stop forward or backward movement
				MotorA.stop(true);
				MotorB.stop(true);
				break;
			case 1: // forward
				MotorA.backward();
				MotorB.backward();
				break;
			case 2: // backward
				MotorA.forward();
				MotorB.forward();
				break;
			case 3: // stop: left, right movement
				MotorA.stop(true);
				MotorB.stop(true);
				break;
			case 4: // left
				MotorA.backward();
				MotorB.forward();
				break;
			case 5: // right
				MotorA.forward();
				MotorB.backward();
				break;
			case 6: // record intersection
				intersections.add(getWaypoint());
				break;
			case 7: // record office
				offices.add(getWaypoint());
				break;
			case 8: // exit
				exit = true;
			case -1:
				exit = true;
			default:
				break;
			}
		}
		// when done return to start state
		//startState();
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
	
	private static void bluetoothTest(){
		boolean exit = false;
		int command = 1;
		while(!exit){
			try{
				command = readBTInput();
				if(command == 0|| command == -1){
					exit = true;
					closeConn();
					System.out.println("Goodbye!");
				}else{
					System.out.println(command);
					Delay.msDelay(500);
				}
			
			}catch(Exception e){
				//no command read
			}
		}
	}
	
	private static int readBTInput(){
		try {
			//Delay.msDelay(60);
			return in.read();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.print("Read Error");
			return 0;
		}
	}
	
	private static void closeConn(){
		if(in != null){
			try {
				in.close();
				btConn.close();
			} catch (IOException e) {
				
			}
		}
	}
	
	private static void test(){
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
