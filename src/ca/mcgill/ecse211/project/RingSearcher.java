package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensors.SensorData;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class helps our robot to search for rings on a grid
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class RingSearcher{
  private static final int ACCELERATION = 300;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private Navigation navigation;
  private Odometer odometer;
  private SensorData data;


  /**
   * This class provides method to check if there is a ring and if the ring is the color we want
   * 
   * @param nav A Navigation class object instance that controls how our robot moves
   * @param leftMotor A EV3LargeRegulatedMotor object instance that allows control of the left motor
   * @param rightMotor A EV3LargeRegulatedMotor object instance that allows control of the right
   *        motor
   * @throws OdometerExceptions
   */
  public RingSearcher(Navigation nav, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    navigation = nav;
    data = SensorData.getSensorData();
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {this.leftMotor,
        this.rightMotor}) {
      motor.stop();
      motor.setAcceleration(ACCELERATION);
    }
  }

  /**
   * This method turn the robot for certain angle and see if there is a right if there is, it will
   * go to that ring to check its color
   * 
   * @param angle angle to turn in order to check the ring
   * @param target target ring color
   * @return true if it found a ring and the ring has the right color
   */
  public ColorCalibrator.Color search(int angle, ColorCalibrator.Color target) {
    double[] position = odometer.getXYT();
    boolean foundRing = false;
    boolean ColorMatched = false;
    // turn to the angle async
    navigation.turnTo(angle, false, true);
    // if we found a ring, got for the ring and check its color
    // if the color matches, return true
    // if(foundRing) {
    // ColorCalibrator.Color color = goForRingColor();
    // //navigation.travelBackTo(position[0], position[1]);
    // if(color == target) {
    // Sound.twoBeeps();
    // ColorMatched = true;
    // }
    // }else {
    // Sound.beepSequence();;
    // }

    return ColorCalibrator.getColor();
  }
  
  /**
   * *
   * This method retrieve ring from the ring set
   * @return whether retrieved successfully
   */
  public boolean retrieveRing() {
    return true;
  }
}