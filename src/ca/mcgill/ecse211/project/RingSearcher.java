package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.threads.SensorData;
import lejos.hardware.Sound;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class helps our robot to search for rings on a grid
 * itself as a thread will search and retrieve the rings
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class RingSearcher{
  private static final int ROD_RETRIEVE = 70;
  private static final int ROD_PREPARE = 180;
  private static final int SENSOR_ROTATION = -90;
  private static final int ACCELERATION = 3000;
  private static final int ROD_SPEED = 250;
  private static final int SENSOR_SPEED = 50;
  private EV3LargeRegulatedMotor sensorMotor;
  private EV3LargeRegulatedMotor rodMotor;
  private boolean started = false;
  private Odometer odometer;
  private SensorData data;


  /**
   * This class provides method to check if there is a ring and if the ring is the color we want
   * 
   * @param storagenMotor: the motor to move the storage of the robot
   * @param rodMotor: the motor for the rod to collect the ring
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
   * This method searches for the ring and identify its color based using the rod,
   * It will beep based on the color of the ring
   * 
   */
  public void  search() {
    sensorMotor.rotate(SENSOR_ROTATION);
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    sensorMotor.rotate(-SENSOR_ROTATION);

    //determine most frequent colour detected and beep accordingly
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
    Game.INSTANCE.rgbPoller.setStart(true);
  }  
  
  /**
   * This method put the sensor to search rotation to be ready for the searching
   */
  public void prepareSearch() {
    sensorMotor.rotate(SENSOR_ROTATION);
  }
  
  /**
   * This method put the sensor back
   */
  public void resetSearch() {
    sensorMotor.rotateTo(0);
  }
  
  /**
   * This method rotate the rod to a suitable position for retrieve the ring
   */
  public void prepareRetrieve() {
    rodMotor.rotate(ROD_PREPARE);
  }
  
  /**
   * this method retrieve the searched ring
   */
  public void retrieveRing() {
    rodMotor.rotate(ROD_RETRIEVE);
  }
  
  /**
   * Rotate the rod back to the original position
   */
  public void resetRodMotor() {
    rodMotor.rotate(-(ROD_PREPARE+ROD_RETRIEVE));
  }
}
