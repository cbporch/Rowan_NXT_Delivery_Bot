package bot;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.addon.CompassHTSensor;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.robotics.DirectionFinder;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.CompassPilot;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.pathfinding.Path;
import lejos.util.Delay;

@SuppressWarnings("deprecation")
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
	private static float _tileTrackWidth;

	public static void main(String[] args) {
		// initialize Navigator
		_carpetTrackWidth = 15;
		_tileTrackWidth = 16;

		MotorA = new NXTRegulatedMotor(MotorPort.A);
		MotorB = new NXTRegulatedMotor(MotorPort.B);
		pilot = new DifferentialPilot(2.5, _carpetTrackWidth, MotorA, MotorB, false);
		compass = new CompassHTSensor(SensorPort.S1);
		// cPilot = new CompassPilot(compass, 0, 0, MotorA, MotorB, false);
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

		// initialize Bluetooth connection
		if (openConn()) {
			// nav.goTo(0, 0, 180);
			// nav.goTo(0, 0, 0);

			compass.startCalibration();
			pilot.setRotateSpeed(18);
			pilot.rotate(720, true);
			while (pilot.isMoving()) {
			} // wait
			compass.stopCalibration();
			compass.resetCartesianZero();
			// testCompass();

			selectState();
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
			PoseProvider p = nav.getPoseProvider();
			Pose pose = p.getPose();
			pose.setHeading(compass.getDegreesCartesian());
			p.setPose(pose);
			nav.setPoseProvider(p);
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
		System.out.println("Receiving Coordinates");
		if (nav.isMoving()) {
			nav.clearPath();
		}
		while (command > 0) {
			int x = readBTInput(), y = readBTInput(), heading = readBTInput();
			System.out.println();
			nav.addWaypoint(x, y);
			System.out.println("Waypoint Added");
			command = readBTInput();
		}
		// System.out.println("Coord. Received");
		setMotorSpeeds();

		nav.followPath();
		do {
			PoseProvider p = nav.getPoseProvider();
			Pose pose = p.getPose();
			pose.setHeading(compass.getDegreesCartesian());
			p.setPose(pose);
			nav.setPoseProvider(p);
		} while (nav.isMoving());
		return 0;
	}
	
	private static void compassRotate(float heading){
		//while (true) {
			PoseProvider p = nav.getPoseProvider();
			p.getPose().setHeading(compass.getDegreesCartesian());
			nav.setPoseProvider(p);
			if (p.getPose().getHeading() > (heading +0.5) &&
					p.getPose().getHeading() < (heading - 0.5)) {
				pilot.rotate(-(heading - compass.getDegreesCartesian()));
				nav.waitForStop();
			}
		//}
	}

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

	private static void sendCoordinatesBT() {
		// Delay.msDelay(50);
		Pose p = nav.getPoseProvider().getPose();
		float x = p.getX(), y = p.getY(), heading = p.getHeading();
		try {
			String output = Float.toString(x) + "\n" + Float.toString(y) + "\n" + Float.toString(heading) + "\n";
			System.out.println("(" + x + "," + y + "," + heading + ")");
			out.writeBytes(output);
			out.flush();
		} catch (IOException e) {
			// e.printStackTrace();
		}
	}

	private static void testCompass() {
		compass.startCalibration();
		pilot.setRotateSpeed(18);
		pilot.rotate(720, true);
		Delay.msDelay(20000);
		compass.stopCalibration();
		compass.resetCartesianZero();
		// float start = System.currentTimeMillis();
		while (true) {
			System.out.println(compass.getDegreesCartesian());
			PoseProvider p = nav.getPoseProvider();
			p.getPose().setHeading(compass.getDegreesCartesian());
			nav.setPoseProvider(p);
			if (p.getPose().getHeading() != 90.0f) {
				pilot.rotate(-(90 - compass.getDegreesCartesian()));
				// nav.rotateTo(90.0f);
				nav.waitForStop();
			}
		}
	}

	@SuppressWarnings("unused")
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
