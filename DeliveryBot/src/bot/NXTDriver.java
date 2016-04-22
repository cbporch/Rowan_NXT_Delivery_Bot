package bot;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;

import lejos.geom.Point;
import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.addon.CompassHTSensor;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.util.Delay;

public class NXTDriver {
	public static Navigator nav;
	public static DifferentialPilot pilot;
	public static CompassHTSensor compass;
	public static NXTConnection btConn;
	private static DataInputStream in;
	private static DataOutputStream out;
	private static NXTRegulatedMotor MotorA;
	private static NXTRegulatedMotor MotorB;
	private static float _carpetTrackWidth;
	@SuppressWarnings("unused")
	private static float _tileTrackWidth;

	/*
	 * Initializes all objects, calibrates CompassHTSensor, 
	 * and starts Bluetooth connections
	 */
	public static void main(String[] args) {
		// Smaller numbers rotate more
		_carpetTrackWidth = 14.85f;
		_tileTrackWidth = 15.86f;

		MotorA = new NXTRegulatedMotor(MotorPort.A);
		MotorB = new NXTRegulatedMotor(MotorPort.B);
		pilot = new DifferentialPilot(2.5, _carpetTrackWidth, MotorA, MotorB, false);
		compass = new CompassHTSensor(SensorPort.S1);
		nav = new Navigator(pilot);
		setMotorSpeeds();
		Button.ESCAPE.addButtonListener(new ButtonListener() {
			public void buttonPressed(Button b) {
				System.exit(0);
			}

			public void buttonReleased(Button b) {
				System.exit(0);
			}
		});

		System.out.println("\n\nInitializing..");

		// initialize
		if (openConn()) {
			compass.startCalibration();
			pilot.setRotateSpeed(20);
			pilot.rotate(720, true);
			while (pilot.isMoving()) {
				// wait for completion
			}
			compass.stopCalibration();
			Delay.msDelay(50);
			compass.resetCartesianZero();
			Delay.msDelay(50);

			test();
//			selectState();
			closeConn();
		}

		// insert code here to attempt to reconnect to bluetooth?
		System.out.println("Press any button to exit.");
		Button.waitForAnyPress();

	}

	/*
	 * Sets individual motor speeds
	 */
	private static void setMotorSpeeds() {
		MotorA.setSpeed(22);
		MotorB.setSpeed(25);

	}

	/*
	 * Establishes Bluetooth input and output connections
	 */
	public static boolean openConn() {
		try {
			btConn = Bluetooth.waitForConnection(60000, NXTConnection.RAW);
			in = new DataInputStream(btConn.openDataInputStream());
			out = new DataOutputStream(btConn.openOutputStream());
			LCD.clear();
			System.out.println("Connected");
			Sound.beepSequenceUp();
			return true;
		} catch (NullPointerException e) {
			System.out.println("No Connection");
			return false;
		}
	}

	/*
	 * Closes all bluetooth connections
	 */
	private static void closeConn() {
		try {
			in.close();
			out.close();
			btConn.close();
			Sound.beepSequence();
		} catch (IOException e) {
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
			setMotorSpeeds();
			resetHeading();
			switch (command) {
			case 0: // stop forward or backward movement
				MotorA.stop(true);
				MotorB.stop(true);
				nav.stop();
				break;
			case 1: // forward
				nav.getMoveController().forward();
				break;
			case 2: // backward
				nav.getMoveController().backward();
				break;
			case 3: // left
				MotorA.setSpeed(500);
				MotorB.setSpeed(500);
				MotorA.forward();
				MotorB.backward();
				break;
			case 4: // right
				MotorA.setSpeed(500);
				MotorB.setSpeed(500);
				MotorA.backward();
				MotorB.forward();
				break;
			case 5: // send current x and y
				sendCoordinatesBT();
				break;
			case 6: // receive Coordinates
				command = receiveCoordinates();
				if (command == -1) {
					exit = true;
				}
				break;
			case -1:// connection lost
				exit = true;
				break;
			default:
				break;
			}
		}
	}

	/*
	 * Receive Coordinates from Bluetooth connection, add them to a path, then
	 * follow it
	 */
	private static int receiveCoordinates() {
		int command = 1;
		ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
		System.out.println("Receiving Coordinates");
		if (nav.isMoving()) {
			nav.clearPath();
		}
		while (command > 0) {
			int x = readBTInput(), y = readBTInput(), heading = readBTInput();
			// System.out.println();
			waypoints.add(new Waypoint(x, y, heading));
			// nav.addWaypoint(x, y);
			System.out.println("Waypoint Added");
			command = readBTInput();
		}
		setMotorSpeeds();
		navigate(waypoints);
		// nav.followPath();
		// do {
		// PoseProvider p = nav.getPoseProvider();
		// Pose pose = p.getPose();
		// pose.setHeading(compass.getDegreesCartesian());
		// p.setPose(pose);
		// nav.setPoseProvider(p);
		// } while (nav.isMoving());
		return 0;
	}

	/*
	 * Resets heading in PoseProvider, based on data from CompassHTSensor
	 */
	private static void resetHeading() {
		PoseProvider posePr = nav.getPoseProvider();
		Pose pose = posePr.getPose();
		pose.setHeading(compass.getDegreesCartesian());
		posePr.setPose(pose);
		nav.setPoseProvider(posePr);
	}

	/*
	 * Navigates a given ArrayList of Waypoint objects
	 */
	private static void navigate(ArrayList<Waypoint> waypoints) {

		for (Waypoint w : waypoints) {
			// returns x| -180<x<180
			resetHeading();
			float goal = nav.getPoseProvider().getPose().relativeBearing(new Point((float) w.getX(), (float) w.getY()));

			System.out.println("heading:\n" + Math.round(nav.getPoseProvider().getPose().getHeading()) 
											+ "\ngoal:\n" + Math.round(goal));
			compassRotateToWaypoint(w);
			nav.goTo((float)w.getX(), (float)w.getY());
			nav.waitForStop();
		}
		Sound.twoBeeps();
	}

	/*
	 * Rotates to an exact heading between 0 and 359.9
	 */
	private static void compassRotateTo(float goal) {
		System.out.println("turning to:\n" + Math.round(goal));
		PoseProvider posePr = nav.getPoseProvider();
		while ((compass.getDegreesCartesian() > (goal + 1)) || (compass.getDegreesCartesian() < (goal - 1))) {
			resetHeading();
			System.out.println(Math.round(compass.getDegreesCartesian())
								+ " " + Math.round(goal)
								+ " " + Math.round((posePr.getPose().getHeading() - goal)));		
			if (posePr.getPose().getHeading() != goal) {
				float turnDegrees = (posePr.getPose().getHeading() - goal);
				if (turnDegrees < -180) {
					turnDegrees = 360 + turnDegrees;
				} else if (turnDegrees > 180) {
					turnDegrees = turnDegrees - 360;
				}
				pilot.rotate(turnDegrees);
				nav.waitForStop();
			}
		}
	}

	/*
	 * Rotates to point in the direction of a given waypoint.
	 */
	private static void compassRotateToWaypoint(Waypoint w){
		PoseProvider posePr = nav.getPoseProvider();
		Point goal = new Point((float)w.getX(), (float)w.getY());
		resetHeading();
		// while current heading is not within 2 degrees of angle to goal
		float angleTo = posePr.getPose().angleTo(goal);
		if(angleTo < 0){
			angleTo = 360 + posePr.getPose().angleTo(goal);
		}else{
			angleTo = posePr.getPose().angleTo(goal);
		}
		while((angleTo < compass.getDegreesCartesian() - 1.0f) ||
				(angleTo > compass.getDegreesCartesian() + 1.0f)){
			resetHeading();
			System.out.println(Math.round(compass.getDegreesCartesian())
					+ " " + Math.round(angleTo)
					+ " " + Math.round(-posePr.getPose().relativeBearing(goal)));
			
			pilot.rotate(-posePr.getPose().relativeBearing(goal));
			while(pilot.isMoving()){
				if(compass.getDegreesCartesian() == angleTo){
					pilot.stop();
				}
			}
			if(posePr.getPose().angleTo(goal) < 0){
				angleTo = 360 + posePr.getPose().angleTo(goal);
			}else{
				angleTo = posePr.getPose().angleTo(goal);
			}
		}
	}
	
	/*
	 * Reads Bluetooth input and returns an int
	 */
	private static int readBTInput() {
		try {
			int out = in.readInt();
			System.out.print(out + " ");
			return out;
		} catch (IOException e) {
			System.out.println("Read Error");
			return -1;
		}
	}

	/*
	 * Sends current pose information over Bluetooth
	 */
	private static void sendCoordinatesBT() {
		// Delay.msDelay(50);
		Pose p = nav.getPoseProvider().getPose();
		float x = p.getX(), y = p.getY(), heading = p.getHeading();
		try {
			// Current x, y, heading
			String output = Float.toString(x) + "\n" + Float.toString(y) + "\n" + Float.toString(heading) + "\n";
			System.out.println("(" + x + "," + y + "," + heading + ")");
			out.writeBytes(output);
			out.flush();
		} catch (IOException e) {
			// e.printStackTrace();
		}
	}

	/*
	 * Tests compassRotateTo method. Should rotate NXT to 90 degrees and maintain heading
	 */
	@SuppressWarnings("unused")
	private static void testCompass() {
		while (true) {
			compassRotateTo(90);
		}
	}

	/*
	 * Tests compassRotateTo method. Should turn 90 degrees, beep twice, then continue
	 * that pattern. (Turn, beep, turn, beep, etc.)
	 */
	@SuppressWarnings("unused")
	private static void testCompassRotate() {
		do {
			System.out.println("Start test");
			compassRotateTo(0);
			Sound.twoBeeps();
			compassRotateTo(90);
			Sound.twoBeeps();
			compassRotateTo(180);
			Sound.twoBeeps();
			compassRotateTo(270);
			Sound.twoBeeps();
		} while (true);
	}

	/*
	 * Tests navigate method. Should drive in a square continuously.
	 */
	@SuppressWarnings("unused")
	private static void test() {
		 ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
		 waypoints.add(new Waypoint(0,20));
		 waypoints.add(new Waypoint(20,20));
		 waypoints.add(new Waypoint(20,0));
		 waypoints.add(new Waypoint(0,0));
		
		 while(true) {
			 navigate(waypoints);
		 }
//		while (true) {
//			resetHeading();
//			System.out.println(nav.getPoseProvider().getPose().getHeading());
//		}
	}

	/*
	 * Tests readBTInput method.
	 */
	@SuppressWarnings("unused")
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
}
