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
  private EV3LargeRegulatedMotor storageMotor;
  private EV3MediumRegulatedMotor rodMotor;
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
  public RingSearcher(EV3LargeRegulatedMotor storageMotor, EV3MediumRegulatedMotor rodMotor)
      throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.storageMotor = storageMotor;
    this.rodMotor = rodMotor;
    data = SensorData.getSensorData();
    for (BaseRegulatedMotor motor : new BaseRegulatedMotor[] {this.storageMotor, this.rodMotor}) {
      motor.stop();
      motor.setAcceleration(ACCELERATION);
    }
  }

  /**
   * This method searches for the ring and identify its color based using the rod,
   * It will beed based on the color of the ring
   * 
   */
  private void  search() {
    double[] position = odometer.getXYT();
    // turn to the angle async

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
  }

  /**
   * this method retrieve the searched ring
   */
  public void retrieveRing() {
    storageMotor.rotateTo(-45);
    rodMotor.rotateTo(-70);
    rodMotor.rotateTo(0);
  }

  protected void runMethod() {
    retrieveRing();

  }
}
