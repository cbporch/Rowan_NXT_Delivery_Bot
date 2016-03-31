package bot;

import java.util.ArrayList;

import lejos.nxt.NXTRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;

public class DiffPilot extends DifferentialPilot {

	
	public DiffPilot(double leftWheelDiameter, double rightWheelDiameter, double trackWidth, RegulatedMotor leftMotor,
			RegulatedMotor rightMotor, boolean reverse) {
		super(leftWheelDiameter, rightWheelDiameter, trackWidth, leftMotor, rightMotor, reverse);
		
		_parity = (byte) (reverse ? -1 : 1);
	}

	public DiffPilot(double wheelSize, double trackWidth, NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor, boolean reverse) {
		super(wheelSize, trackWidth, leftMotor, rightMotor, reverse);

	    _parity = (byte) (reverse ? -1 : 1);
	}

	private void setSpeed(final float _leftMotorTravelSpeed2, final float _rightMotorTravelSpeed2)
	  {
	    _left.setSpeed((int) _leftMotorTravelSpeed2);
	    _right.setSpeed((int) _rightMotorTravelSpeed2);
	  }
	
	public void setTravelSpeed(double travelSpeedLeft, double travelSpeedRight) {
	    _leftMotorTravelSpeed = (int)Math.round(travelSpeedLeft * (_leftDegPerDistance));
	    _rightMotorTravelSpeed = (int)Math.round(travelSpeedRight * (_rightDegPerDistance));
	    setSpeed(_leftMotorTravelSpeed, _rightMotorTravelSpeed);
	  
	}
	
	@Override
	public void setAcceleration(int acceleration){
		this.setAcceleration(acceleration, acceleration);
	}
	
	public void setAcceleration(int leftAcceleration, int rightAcceleration) {
		int _leftAcceleration = leftAcceleration;
		int _rightAcceleration = rightAcceleration;
		setMotorAccel(_leftAcceleration, _rightAcceleration);
	}
	
	private void setMotorAccel(int leftAccel, int rightAccel){
		int rightMotorAccel  = (int)Math.round(0.5 * rightAccel * (2 * _rightDegPerDistance));
		int leftMotorAccel  = (int)Math.round(0.5 * leftAccel * (2 * _leftDegPerDistance));
		
		_left.setAcceleration(leftMotorAccel);
	    _right.setAcceleration(rightMotorAccel);
	}
	
	public void travel(final double distance, final boolean immediateReturn)
	  {
	    _type = Move.MoveType.TRAVEL;
	    _distance = distance;
	    _angle = 0;
	    if (distance == Double.POSITIVE_INFINITY)
	    {
	      forward();
	      return;
	    }
	    if ((distance == Double.NEGATIVE_INFINITY))
	    {
	      backward();
	      return;
	    }
	    movementStart(immediateReturn);
	    setSpeed(Math.round(_leftMotorTravelSpeed * _leftDegPerDistance), Math.round(_rightMotorTravelSpeed * _rightDegPerDistance));
	    _left.rotate((int) (_parity * distance * _leftDegPerDistance), true);
	    _right.rotate((int) (_parity * distance * _rightDegPerDistance),
	            immediateReturn);
	    if (!immediateReturn) waitComplete();
	  }
	
	   protected void movementStart(boolean alert)
	  {
	    if (isMoving())  movementStop();
	    reset();
	    for(MoveListener ml : _listeners)
	      ml.moveStarted(new Move(_type,
	            (float) _distance, (float) _angle, 
	            (float) getTravelSpeed(), _robotRotateSpeed, isMoving()), this);
	  }

	  /**
	   * called by Arc() ,travel(),rotate(),stop() rotationStopped()
	   * calls moveStopped on listener
	   */
	  private synchronized void movementStop()
	  {
		  
	    for(MoveListener ml : _listeners)
	      ml.moveStopped(new Move(_type,
	            getMovementIncrement(), getAngleIncrement(), 
	            (float) getTravelSpeed(), _robotRotateSpeed, isMoving()), this);
	  }
	
	private void waitComplete()
	  {
	    while(isMoving())
	    {
	      _left.waitComplete();
	      _right.waitComplete();
	    }
	  }
	
	private float _leftMotorTravelSpeed;
	private float _rightMotorTravelSpeed;
	private byte _parity;
	private double _distance;
	private double _angle;
	private ArrayList<MoveListener> _listeners= new ArrayList<MoveListener>();
	private float _robotRotateSpeed;
}
