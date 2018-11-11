package ca.mcgill.ecse211.threads;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
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
public class RingSearcher extends ThreadControl {
  private static final int ACCELERATION = 300;
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
    rodMotor.setSpeed(250);
    rodMotor.setAcceleration(3000);
    this.sensorMotor.setSpeed(50);
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
    sensorMotor.rotate(-170);
    sensorMotor.rotate(170);
  }

  public void prepareRetrieve() {
    rodMotor.rotate(180);
  }
  /**
   * this method retrieve the searched ring
   */
  public void retrieveRing() {
    rodMotor.rotate(60);
    rodMotor.rotate(-60);
    rodMotor.rotate(180);
  }

  protected void runMethod() {
    search();
    retrieveRing();

  }
}
