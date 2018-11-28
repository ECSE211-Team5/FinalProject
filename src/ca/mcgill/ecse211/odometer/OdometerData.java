package ca.mcgill.ecse211.odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This class stores and provides thread safe access to data required used by the Odometer classes.
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class OdometerData {
  // Position parameters
  private volatile double x; // x-axis position
  private volatile double y; // y-axis position
  private volatile double theta; // Head angle

  // Class control variables
  private volatile static int numberOfIntances = 0; // Number of OdometerData
                                                    // objects instantiated
                                                    // so far
  private static final int MAX_INSTANCES = 1; // Maximum number of
                                              // OdometerData instances

  // Thread control tools
  private static Lock lock = new ReentrantLock(true); // Fair lock for
                                                      // concurrent writing
  private volatile boolean isReseting = false; // Indicates if a thread is
                                               // trying to reset any
                                               // position parameters
  private Condition doneReseting = lock.newCondition(); // Let other threads
                                                        // know that a reset
                                                        // operation is
                                                        // over.

  private static OdometerData odoData = null;

  /**
   * This is the class constructor for the OdometerData class. It cannot be instantiated externally.
   * A factory is used instead such that only one instance of this class is ever created.
   */
  protected OdometerData() {
    this.x = 0;
    this.y = 0;
    this.theta = 0;
  }

  /**
   * This method returns an OdometerData instance and makes sure that only one instance is ever
   * created.
   * 
   * @return An OdometerData object
   * @throws OdometerExceptions
   */
  public synchronized static OdometerData getOdometerData() throws OdometerExceptions {
    if (odoData != null) { // Return existing object
      return odoData;
    } else if (numberOfIntances < MAX_INSTANCES) {
      // create object and return it
      odoData = new OdometerData();
      numberOfIntances += 1;
      return odoData;
    } else {
      throw new OdometerExceptions("Only one intance of the Odometer can be created.");
    }
  }

  /**
   * This method returns the Odometer data consisting of the current x, y coordinates and angle
   * (theta).
   * 
   * @return The odometer data.
   */
  public double[] getXYT() {
    double[] position = new double[4];
    lock.lock();
    try {
      while (isReseting) { // If a reset operation is being executed, wait
        // until it is over.
        doneReseting.await(); // Using await() is lighter on the CPU
        // than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;

    } catch (InterruptedException e) {
      // Print exception to screen
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;
  }

  /**
   * This method adds dx, dy and dtheta to the current values of x, y and theta, respectively.
   * 
   * @param dx The change in x coordinate to add to the current x coordinate
   * @param dy The change in y coordinate to add to the current y coordinate
   * @param dtheta The change in angle to add to the current angle (theta)
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isReseting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates
                                                    // within 360
                                                    // degrees
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }

  }

  /**
   * This method overrides the values of x, y and theta. Used for odometry correction.
   * 
   * @param x The value to set the current x coordinate to
   * @param y The value to set the current y coordinate to
   * @param theta The value to set the current angle (theta) to
   */
  public void setXYT(double x, double y, double theta) {
    lock.lock();
    isReseting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * This method overrides the x coordinate. Used for odometry correction.
   * 
   * @param x The value to set the current x coordinate to
   */
  public void setX(double x) {
    lock.lock();
    isReseting = true;
    try {
      this.x = x;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * This method overrides the y coordinate. Used for odometry correction.
   * 
   * @param y The value to set the current y coordinate to
   */
  public void setY(double y) {
    lock.lock();
    isReseting = true;
    try {
      this.y = y;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * This method overrides theta. Used for odometry correction.
   * 
   * @param theta The value to set the current angle (theta) to
   */
  public void setTheta(double theta) {
    lock.lock();
    isReseting = true;
    try {
      this.theta = theta;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }
}
