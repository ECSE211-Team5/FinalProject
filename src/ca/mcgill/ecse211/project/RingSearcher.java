package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensors.SensorData;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

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
public class RingSearcher extends Thread implements ThreadControl {
  private static final int ACCELERATION = 300;
  private EV3LargeRegulatedMotor storageMotor;
  private EV3MediumRegulatedMotor rodMotor;
  private boolean started = false;
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
   * This method turn the robot for certain angle and see if there is a right if there is, it will
   * go to that ring to check its color
   * 
   * @param angle angle to turn in order to check the ring
   * @param target target ring color
   * @return true if it found a ring and the ring has the right color
   */
  public ColorCalibrator.Color search(ColorCalibrator.Color target) {
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

    return ColorCalibrator.getColor();
  }

  private void retrieveRing() {
    storageMotor.rotateTo(-45);
    rodMotor.rotateTo(-70);
    rodMotor.rotateTo(0);
  }

  public synchronized void run() {
    try {
      while (true) {
        if (!started) {
          wait();
        } else {
          retrieveRing();
          wait(100);
        }
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

  }

  @Override
  public synchronized boolean isStarted() {
    return started;
  }

  @Override
  public synchronized void setStart(boolean start) {
    started = start;

    if (started)
      notify();
  }
}
