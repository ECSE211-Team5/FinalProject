package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.project.Game;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class implements correction for the odometry on our robot using a light sensor.
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double TILE_WIDTH = 30.48;
  private static final double LINE_COLOR_THRESHOLD = 0.35;
  private static final double SENSOR_DIS = 15.5;
  private Odometer odometer;
  private static final SensorModes myColor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
  private static SampleProvider myColorSample = myColor.getMode("Red");
  private static float[] sampleColor = new float[myColor.sampleSize()];

  /**
   * This is the class constructor for the OdometryCorrection class.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {
    // Utilize the singleton Odometer object instance for thread safety.
    this.odometer = Odometer.getOdometer();
  }

  /**
   * This method is called when this OdometryCorrection object instance is started as a thread.
   * Functionality wise it will correct the rotation and position of the robot once a black line is
   * detected.
   * 
   * @throws OdometerExceptions
   */
  public void run() {
    long correctionStart, correctionEnd;
    boolean onTopOfLine = false;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // Fetch the sample at offset 0
      myColorSample.fetchSample(sampleColor, 0);

      // Check if our light sensor has read a black line and is not already on top of one
      if (sampleColor[0] < LINE_COLOR_THRESHOLD && !onTopOfLine) {

        // New black line detected
        Sound.beep();
        onTopOfLine = true;

        double x = odometer.getXYT()[0];
        double y = odometer.getXYT()[1];

        if (Math.abs(x % TILE_WIDTH) < Math.abs(y % TILE_WIDTH)) {
          odometer.setX(Math.round(x / TILE_WIDTH) * TILE_WIDTH);
        } else {
          odometer.setY(Math.round(y / TILE_WIDTH) * TILE_WIDTH);
        }

      } else if (sampleColor[0] > LINE_COLOR_THRESHOLD) {
        // No longer on top of line, reset to false
        onTopOfLine = false;
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }

  /**
   * This method corrects our robot's Odometer readings
   * 
   * @param angle The current angle that the robot is facing
   */
  public void doCorrection(double angle) {
    double[] position = odometer.getXYT();
    // Check that our robot's angle is within certain bounds and correct odometer if required.
    if (angle < 5 || angle > 355) {
      int sensorCoor = (int) Math.round(position[1] - SENSOR_DIS / Game.TILE);
      odometer.setY(sensorCoor + SENSOR_DIS / Game.TILE);
    } else if (angle < 185 && angle > 175) {
      int sensorCoor = (int) Math.round(position[1] + SENSOR_DIS / Game.TILE);
      odometer.setY(sensorCoor - SENSOR_DIS / Game.TILE);
    } else if (angle < 95 && angle > 85) {
      int sensorCoor = (int) Math.round(position[0] - SENSOR_DIS / Game.TILE);
      odometer.setX(sensorCoor + SENSOR_DIS / Game.TILE);
    }
  }
}
