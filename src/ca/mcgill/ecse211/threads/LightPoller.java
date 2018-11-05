package ca.mcgill.ecse211.threads;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.robotics.SampleProvider;

/**
 * This class implements the Light Sensor Poller for our robot
 * it runs pulls the sensor data every 50 miliseconds
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class LightPoller extends ThreadControl{
  protected SampleProvider us[];
  protected SensorData cont;
  protected float[][] lgData;
  protected float lastValue[];
  protected static int WAIT_TIME = 50;
  private int id;
  private static int sensorNumber = 0;

  /**
   * This constructor creates an instance of the LightPoller class to provide distance data from an
   * light sensor to our robot.
   * 
   * @param us a SampleProvider class instance that helps us to store an array of ultrasonic sensor
   *        data.
   * @param lgData an array of distance data to be used by our Wall Follower's
   *        UltrasonicControllers.
   * @param cont a BangBangController or PController instance that has accumulated distance data
   *        stored in usData passed to it.
   * @throws OdometerExceptions
   */
  public LightPoller(SampleProvider[] us, float[][] lgData, SensorData cont) throws OdometerExceptions {
    this.us = us;
    this.cont = cont;
    this.lgData = lgData;
    this.id = sensorNumber;
    isStarted = true;
    lastValue = new float[2];
    sensorNumber++;
  }

  /**
   * the run method to be performed in run method, collect light data
   */
  protected void runMethod() {
    double l[] = new double[2];
    for(int i = 0; i < us.length; i++) {
      us[i].fetchSample(lgData[i], 0); // acquire data
  
      int distance = (int) (lgData[i][0] * 100); // extract from buffer, multiply by 100 for convenience
                                              // and allow it to be cast to int
      l[i] = distance - lastValue[i]; // now take action depending on value
      lastValue[i] = distance; 
    }
    cont.setL(l);
  }
}
