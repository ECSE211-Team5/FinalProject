package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.project.Game;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements odometry on our robot.
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class Odometer extends OdometerData implements Runnable {
  private OdometerData odoData;
  private static Odometer odometer = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private double TRACK;
  private double WHEEL_RAD;

  private double[] position;

  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the class constructor for the Odometer class. It cannot be instantiated externally.
   * 
   * @param leftMotor An EV3LargeRegularedMotor object instance that allows control of the left
   *        motor
   * @param rightMotor An EV3LargeRegularedMotor object instance that allows control of the right
   *        motor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
  }

  /**
   * This method ensures that only one instance of the Odometer object is used throughout the code.
   * 
   * @param leftMotor An EV3LargeRegularedMotor object instance that allows control of the left
   *        motor
   * @param rightMotor An EV3LargeRegularedMotor object instance that allows control of the right
   *        motor
   * @return A new or pre-existing Odometer object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odometer != null) { // Return existing object
      return odometer;
    } else { // create object and return it
      odometer = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odometer;
    }
  }

  /**
   * This method returns a pre-existing Odometer object instance. It is meant to be used only if an
   * Odometer object has already been created beforehand.
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {
    if (odometer == null) {
      throw new OdometerExceptions("No previous Odometer exits.");
    }
    return odometer;
  }

  /**
   * This method is called when our Odometer object is started as a thread and begins to keep track
   * of motor rotations
   */
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Calculate new robot position based on tachometer counts
      double distL, distR, deltaD, deltaT, dX, dY;
      int nowTachoL, nowTachoR;
      position = odometer.getXYT();

      // Calculate the change in distances and Theta with motor tacho counts
      nowTachoL = leftMotor.getTachoCount();
      nowTachoR = rightMotor.getTachoCount();
      distL = 3.14159 * WHEEL_RAD * (nowTachoL - leftMotorTachoCount) / 180;
      distR = 3.14159 * WHEEL_RAD * (nowTachoR - rightMotorTachoCount) / 180;
      leftMotorTachoCount = nowTachoL;
      rightMotorTachoCount = nowTachoR;
      deltaD = 0.5 * (distL + distR);
      deltaT = (distL - distR) / TRACK;

      double Theta = Math.toRadians(position[2]);
      Theta += deltaT;
      dX = deltaD * Math.sin(Theta);
      dY = deltaD * Math.cos(Theta);

      // Update odometer values with new calculated values
      odometer.update(dX / Game.TILE, dY / Game.TILE, Math.toDegrees(deltaT));

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }
}
