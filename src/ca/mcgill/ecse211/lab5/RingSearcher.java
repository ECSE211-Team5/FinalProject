package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensors.SensorData;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class RingSearcher {
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 50;
	private static final int RING_DISTANCE = 30;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Navigation navigation;
	private Odometer odometer;
	private SensorData data;

	/**
	 * This class provides method to check if there is a ring and if the ring is the color
	 * we want
	 * @param nav
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	public RingSearcher(Navigation nav, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor)
			throws OdometerExceptions {
		this.odometer = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		navigation = nav;
		data = SensorData.getSensorData();
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor }) {
			motor.stop();
			motor.setAcceleration(300);
		}
	}

	/**
	 * This method turn the robot for certain angle and see if there is a right
	 * if there is, it will go to that ring to check its color
	 * @param angle: angle to turn in order to check the ring
	 * @param target: target color
	 * @return: true if it found a ring and the ring has the right color
	 */
	public boolean search(int angle, ColorCalibrator.Color target) {
		double[] position = odometer.getXYT();
		boolean foundRing = false;
		boolean ColorMatched = false;
		//turn to the angle async 
		navigation.turnTo(angle, true);
		//while turnninng, check the value from the us sensor to see if there is a ring
		while(leftMotor.isMoving() || rightMotor.isMoving()) {
			if(ColorCalibrator.getColor() == target) {
				navigation.stop();
				Sound.twoBeeps();
				foundRing = true;
				break;
			}else if(ColorCalibrator.getColor() != ColorCalibrator.Color.Other) {
				Sound.beep();
				navigation.stop();
				break;
			}
		}
		//if we found a ring, got for the ring and check its color 
		//if the color matches, return true
//		if(foundRing) {
//			ColorCalibrator.Color color = goForRingColor();
//			//navigation.travelBackTo(position[0], position[1]);
//			if(color == target) {
//				Sound.twoBeeps();
//				ColorMatched = true;
//			}
//		}else {
//			Sound.beepSequence();;
//		}
		
		return foundRing;
	}

	/**
	 * This method moves the car to the ring and check its color
	 * @return the color of the ring
	 */
	private ColorCalibrator.Color goForRingColor() {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		ColorCalibrator.Color color;
		while((color=ColorCalibrator.getColor()) == ColorCalibrator.Color.Other) {
			leftMotor.forward();
			rightMotor.forward();
		}
		//stop the motor when find the rings
		navigation.stop();
		return color;
	}
}
