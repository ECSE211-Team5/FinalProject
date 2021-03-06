package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.Navigation;
import ca.mcgill.ecse211.threads.SensorData;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class helps our robot to localize itself using the ultrasonic sensor
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class UltrasonicLocalizer {
  private static final int ROTATE_SPEED = 600;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private Odometer odometer;
  private SensorData data;
  private Navigation navigation;

  private static final int READINGS_THRESHOLD = 40;
  private static final double wallDistance = 40.0;
  private static final double wallDistanceError = 5;

  /**
   * This is the class constructor for the UltrasonicLocalizer class
   * 
   * @param leftMotor An EV3LargeRegularedMotor object instance that allows control of the left
   *        motor
   * @param rightMotor An EV3LargeRegularedMotor object instance that allows control of the right
   *        motor
   * @throws OdometerExceptions
   */
  public UltrasonicLocalizer(Navigation nav, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.data = SensorData.getSensorData();
    this.navigation = nav;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  /**
   * This method decides whether to use rising or falling edge localization based on which button is
   * pressed on our EV3 brick.
   * 
   * @param buttonChoice The left or right button on the EV3 brick
   */
  public void localize(int buttonChoice) {
    if (buttonChoice == Button.ID_RIGHT) {
      risingEdge();
    } else {
      fallingEdge();
    }
  }

  /**
   * This method helps our robot find north by using rising edge localization. We assume the robot
   * starts by facing left of the x-axis wall.
   */
  private void risingEdge() {
    double angle1, angle2, theta = 0;
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // 1. turn until no wall, then see wall and check angle1
    leftMotor.forward();
    rightMotor.backward();

    turnToWall();
    turnAwayFromWall();
    // Sound.beep();
    angle1 = odometer.getXYT()[2];

    // 2. turn opposite way, until no wall, then see wall and check angle2
    rightMotor.forward();
    leftMotor.backward();

    turnToWall();
    turnAwayFromWall();
    // Sound.beep();
    angle2 = odometer.getXYT()[2];

    // 3. Turn to calculate angle, then turn to face y-axis, wait for button press
    int lowerAngleBound = -222, upperAngleBound = -46;
    if (angle1 < angle2)
      theta = lowerAngleBound + (angle1 + angle2) / 2;
    else
      theta = upperAngleBound + (angle1 + angle2) / 2;
    navigation.turnTo(theta + 180);
    odometer.setTheta(0);
    Button.waitForAnyPress();
  }

  /**
   * This method helps our robot find north by using rising edge localization. We assume the robot
   * starts by facing the top of the y-axis wall.
   */
  private void fallingEdge() {
    double angle1, angle2, theta = 0;
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // 1. Turn right until we see the first wall, then turn left
    leftMotor.forward();
    rightMotor.backward();

    turnAwayFromWall();
    turnToWall();
    // Sound.beep();
    angle1 = odometer.getXYT()[2];

    // 2. Turn left until we see the second wall, then turn right
    navigation.turn(-90);
    rightMotor.forward();
    leftMotor.backward();

    turnAwayFromWall();
    turnToWall();
    // Sound.beep();
    angle2 = odometer.getXYT()[2];

    // 3. Calculate the angle and face y-axis 0
    int lowerAngleBound = -222, upperAngleBound = -45;
    if (angle1 < angle2)
      theta = lowerAngleBound + (angle1 + angle2) / 2;
    else
      theta = upperAngleBound + (angle1 + angle2) / 2;
    navigation.turnTo(theta);
    odometer.setTheta(0);
    navigation.setSlowAcc();
    // wait for any button press
  }


  /**
   * This method keeps turning our robot towards a wall until it has detected one. We takes 10
   * readings to be sure.
   */
  private void turnToWall() {
    int numberOfReadings = 0;

    while (numberOfReadings < READINGS_THRESHOLD) {
      if (data.getD() < wallDistance - wallDistanceError)
        numberOfReadings++;
    }
  }

  /**
   * This method keeps turning our robot away from a wall until it has detected no wall. We takes 10
   * readings to be sure.
   */
  private void turnAwayFromWall() {
    int numberOfReadings = 0;

    while (numberOfReadings < READINGS_THRESHOLD) {
      if (data.getD() > wallDistance + wallDistanceError)
        numberOfReadings++;
    }
  }
}
