package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.Game;
import ca.mcgill.ecse211.project.Navigation;
import ca.mcgill.ecse211.threads.SensorData;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class helps our robot to localize itself using the light sensor
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class LightLocalizer {
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

	private Odometer odometer;
	private SensorData data;
	private Navigation navigation;
	private static final int FORWARD_SPEED = 100;
	private static final double SENSOR_DIS = 16.3;
	private static final int blackLineColor = -5;
  /**
   * This is the class constructor
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  public LightLocalizer(Navigation nav, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.data = SensorData.getSensorData();
    this.navigation = nav;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  /**
   * (*Improve*)
   *  Once the robot know what angle it is facing, this method looks for the x,y axis origins knowing
   * it is in the first tile facing north.
   * @param sC: the coordinate to set to after localization
   */
  public void localize(int[] sC) {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // 1. GO forward find the y=0 line
    leftMotor.forward();
    rightMotor.forward();
    while (leftMotor.isMoving() || rightMotor.isMoving()) {
      if (data.getL()[0] < blackLineColor) {
        leftMotor.stop(true);
      }
      if (data.getL()[1] < blackLineColor) {
        rightMotor.stop(true);
      }
    }
    odometer.setTheta(0.0);
    Sound.beep();
    odometer.setY(0);
    // 2. Turn and go forward find the x=0 line
    navigation.turnTo(90);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    while (leftMotor.isMoving() || rightMotor.isMoving()) {
      if (data.getL()[0] < blackLineColor) {
        leftMotor.stop(true);
      }
      if (data.getL()[1] < blackLineColor) {
        rightMotor.stop(true);
      }
    }
    odometer.setTheta(90.0);
    Sound.beep();
    odometer.setX(0);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    // 3. Go backwards by sensor-wheel center distance in x-direction
    leftMotor.rotate(Navigation.convertDistance(Game.WHEEL_RAD, Game.SEN_DIS), true);
    rightMotor.rotate(Navigation.convertDistance(Game.WHEEL_RAD, Game.SEN_DIS), false);
    // 4. Go backwards by sensor-wheel center distance in y-direction
    navigation.turnTo(0);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    //double sensorDistanceOffset = 2.5;
    leftMotor.rotate(
        Navigation.convertDistance(Game.WHEEL_RAD, Game.SEN_DIS),
        true);
    rightMotor.rotate(
        Navigation.convertDistance(Game.WHEEL_RAD, Game.SEN_DIS),
        false);
    odometer.setTheta(0);
    odometer.setX(sC[0]);
    odometer.setY(sC[1]);

  }
}
