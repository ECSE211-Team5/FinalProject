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
	private static final int FORWARD_SPEED = 200;
	private static final double SENSOR_DIS = 16.3;
	private static final int blackLineColor = 20;
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
    while (data.getL()[0] > blackLineColor);
    Sound.beep();
    odometer.setY(0);
    // 2. Turn and go forward find the x=0 line
    navigation.turnTo(90);
    leftMotor.setSpeed(FORWARD_SPEED / 2);
    rightMotor.setSpeed(FORWARD_SPEED / 2);
    leftMotor.forward();
    rightMotor.forward();
    while (data.getL()[0] > blackLineColor);
    Sound.beep();
    odometer.setX(0);
    leftMotor.setSpeed(FORWARD_SPEED / 2);
    rightMotor.setSpeed(FORWARD_SPEED / 2);
    // 3. Go backwards by sensor-wheel center distance in x-direction
    leftMotor.rotate(Navigation.convertDistance(Game.WHEEL_RAD, -SENSOR_DIS), true);
    rightMotor.rotate(Navigation.convertDistance(Game.WHEEL_RAD, -SENSOR_DIS), false);
    // 4. Go backwards by sensor-wheel center distance in y-direction
    navigation.turnTo(0);
    leftMotor.setSpeed(FORWARD_SPEED / 2);
    rightMotor.setSpeed(FORWARD_SPEED / 2);
    double sensorDistanceOffset = 2.5;
    leftMotor.rotate(
        Navigation.convertDistance(Game.WHEEL_RAD, -SENSOR_DIS - sensorDistanceOffset),
        true);
    rightMotor.rotate(
        Navigation.convertDistance(Game.WHEEL_RAD, -SENSOR_DIS - sensorDistanceOffset),
        false);
    odometer.setTheta(0);
    odometer.setX(sC[0]);
    odometer.setY(sC[1]);

  }
}
