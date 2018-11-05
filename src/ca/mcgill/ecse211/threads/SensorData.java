package ca.mcgill.ecse211.threads;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * This class implements methods to manage data from our sensors
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class SensorData {
  // Sensor data parameters
  private double[] lights; // Head angle
  private volatile double distance;
  private volatile double angle;
  private int rgb[];

  // Class control variables
  private volatile static int numberOfIntances = 0; // Number of OdometerData
                                                    // objects instantiated
                                                    // so far
  private static final int MAX_INSTANCES = 1; // Maximum number of
                                              // OdometerData instances

  // Thread control tools
  private static Lock lightLock = new ReentrantLock(true); // Fair lock for concurrent writing light sensor data
  
  private static Lock rgbLock = new ReentrantLock(true); // Fair lock for concurrent writing rgb sensor data

  private static SensorData sensorData = null;

  /**
   * Default constructor. The constructor is private. A factory is used instead such that only one
   * instance of this class is ever created.
   */
  protected SensorData() {
    // Default distance value is 40 cm from any walls.
    this.distance = 40;
    // Default light value is 0
    this.lights = new double[2];
    rgb = new int[3];
    for (int j = 0; j < rgb.length; j++) {
      rgb[j] = 0;
    }
  }

  /**
   * OdometerData factory. Returns an OdometerData instance and makes sure that only one instance is
   * ever created. If the user tries to instantiate multiple objects, the method throws a
   * MultipleOdometerDataException.
   * 
   * @return An OdometerData object
   * @throws OdometerExceptions
   */
  public synchronized static SensorData getSensorData() throws OdometerExceptions {
    if (sensorData != null) { // Return existing object
      return sensorData;
    } else if (numberOfIntances < MAX_INSTANCES) { // create object and
                                                   // return it
      sensorData = new SensorData();
      numberOfIntances += 1;
      return sensorData;
    } else {
      throw new OdometerExceptions("Only one intance of the SensorData can be created.");
    }

  }

  /**
   * Return the ultraSonic distance data.
   * 
   * @return the sensor data.
   */
  public double getD() {
    return distance;
  }
  
  /**
   * get the light value data from the two light sensors (protected by lightLock)
   * @return: data from light sensor
   */
  public double[] getL() {    
    //lock the lock for light sensor value
    lightLock.lock();
    try {
      return lights.clone();
    } finally {
      lightLock.unlock();
    }
    
  }

  /**
   * get the rgb data for light sensor (protected by rgb lock)
   * 
   * @return: rgb data
   */
  public int[] getRGB() {
    rgbLock.lock();
    try {
      return rgb.clone();
    }finally {
      rgbLock.unlock();
    }
  }

  /**
   * (deprecated, not using)
   * This method returns the currently stored angle value from the gyro sensor
   * 
   * @return The current angle value
   */
  public double getA() {
    return angle;
  }

  /**
   * This method overwrites the distance value. Use for ultrasonic sensor.
   * 
   * @param d The value to overwrite distance with
   */
  public void setD(double d) {
      this.distance = d;
  }

  /**
   * (deprecated not usings)
   * This method overwrites the angle value.
   * 
   * @param a The value to overwrite angle with
   */
  public void setA(double a) {
      this.angle = a;
  }

  /**
   * set rgb data for color sensor (protected by rgb lock)
   * 
   * @param r: red value
   * @param g: green value
   * @param b: blue value
   */
  public void setRGB(int r, int g, int b) {
    try {
      rgbLock.lock();
      rgb[0] = r;
      rgb[1] = g;
      rgb[2] = b;
    } finally {
      rgbLock.unlock();
    }
  }

  /**
   * This method overwrites the light value. (protected by light lock)
   * 
   * @param l The value to overwrite the current light value with
   */
  public void setL(double l[]) {
    try {
      lightLock.lock();
      this.lights[0] = l[0];
      this.lights[1] = l[1];
    } finally {
      lightLock.unlock();
    }
  }
}
