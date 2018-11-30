package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.threads.SensorData;
import lejos.hardware.Sound;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class helps our robot to search for rings on a grid itself as a thread will search and
 * retrieve the rings
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class RingSearcher {
  private static final int ROD_RETRIEVE = 80;
  private static final int ROD_PREPARE = 90;
  private static final int SENSOR_ROTATION = -100;
  private static final int ACCELERATION = 3000;
  private static final int ROD_SPEED = 250;
  private static final int SENSOR_SPEED = 50;
  private EV3LargeRegulatedMotor sensorMotor;
  private EV3LargeRegulatedMotor rodMotor;
  private boolean started = false;
  private Odometer odometer;
  private SensorData data;


  /**
   * This RingSearcher class constructor checks if there is a ring to collect and records its color
   * 
   * @param sensorMotor An EV3LargeRegulatedMotor object instance that controls a sensor motor
   * @param rodMotor An EV3LargeRegulatedMotor object instance that controls the rod with a light
   *        sensor for ring color detection
   * @throws OdometerExceptions
   */
  public RingSearcher(EV3LargeRegulatedMotor sensorMotor, EV3LargeRegulatedMotor rodMotor)
      throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.sensorMotor = sensorMotor;
    this.rodMotor = rodMotor;
    rodMotor.setSpeed(ROD_SPEED);
    this.sensorMotor.setSpeed(SENSOR_SPEED);
    data = SensorData.getSensorData();
    for (BaseRegulatedMotor motor : new BaseRegulatedMotor[] {this.sensorMotor, this.rodMotor}) {
      motor.stop();
      motor.setAcceleration(ACCELERATION);
    }
  }

  /**
   * This method searches for a ring and rotates the sensorMotor rod
   * 
   * @param angle An integer to rotate the motor for the sensor to
   */
  public void search(int angle) {
    sensorMotor.rotateTo(angle);
  }

  /**
   * This method causes our robot to beep a certain number of times to identify what ring it is
   * picking up
   */
  public void detectColor() {
    Game.INSTANCE.rgbPoller.setStart(true);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    // determine most frequent colour detected and beep accordingly
    Game.INSTANCE.rgbPoller.setStart(false);
    switch (ColorCalibrator.getMostFrequenct()) {
      case Orange:
        Sound.beep();
        Sound.beep();
        Sound.beep();
        Sound.beep();
        break;
      case Yellow:
        Sound.beep();
        Sound.beep();
        Sound.beep();
        break;
      case Green:
        Sound.beep();
        Sound.beep();
        break;
      case Blue:
        Sound.beep();
        break;
      case Other:
        break;
      default:
        break;
    }
  }

  /**
   * This method prepares our robot to search for a ring
   */
  public void prepareSearch() {
    sensorMotor.rotateTo(SENSOR_ROTATION);
  }

  /**
   * This method indicates that our robot is done searching for a ring
   */
  public void finishSearch() {
    sensorMotor.rotateTo(-100);
  }

  /**
   * This method retracts our ring searching rod
   */
  public void resetSearch() {
    sensorMotor.rotateTo(0);
  }

  /**
   * This method rotates the rod to a suitable position to allow a ring to be picked up
   */
  public void prepareRetrieve() {
    rodMotor.rotateTo(ROD_PREPARE);
  }

  /**
   * This method rotates the rod to be ready to retrieve a ring
   */
  public void retrieveRing() {
    rodMotor.rotate(ROD_RETRIEVE);
  }

  /**
   * This method rotates the rod to a safe position
   */
  public void safeRod() {
    rodMotor.rotateTo(180);
  }

  /**
   * This method rotates the rod back to its original position
   */
  public void resetRodMotor() {
    rodMotor.rotateTo(0);
  }
}
