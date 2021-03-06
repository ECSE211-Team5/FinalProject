package ca.mcgill.ecse211.threads;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.ColorCalibrator;
import lejos.robotics.SampleProvider;

/**
 * This class polls a light sensor that is used to detect colored rings.
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class RGBPoller extends LightPoller {

  /**
   * This constructor creates an instance of the RGBPoller class to provide color data from an light
   * sensor to our robot.
   * 
   * @param us A SampleProvider class instance that helps us to store an array of light sensor data.
   * @param usData An array to store light data.
   * @param cont A SensorData object that is used to process color data.
   * @throws OdometerExceptions
   */
  public RGBPoller(SampleProvider us[], float[][] usData, SensorData cont)
      throws OdometerExceptions {
    super(us, usData, cont);
    isStarted = true;
  }

  /**
   * This method reads rgb light sensor data and processes it
   */
  @Override
  protected void runMethod() {
    us[0].fetchSample(lgData[0], 0); // acquire data at offset 0
    // get RGB values from buffer, multiply by 100 for convenience and allow it to be cast to int
    int r = (int) (lgData[0][0] * 100); // extract from buffer, cast to int
    int g = (int) (lgData[0][1] * 100); // extract from buffer, cast to int
    int b = (int) (lgData[0][2] * 100); // extract from buffer, cast to int
    cont.setRGB(r, g, b); // now take action depending on value
    switch (ColorCalibrator.getColor(r, g, b)) {
      case Orange:
        ColorCalibrator.setFrequency(ColorCalibrator.Color.Orange);
        break;
      case Yellow:
        ColorCalibrator.setFrequency(ColorCalibrator.Color.Yellow);
        break;
      case Green:
        ColorCalibrator.setFrequency(ColorCalibrator.Color.Green);
        break;
      case Blue:
        ColorCalibrator.setFrequency(ColorCalibrator.Color.Blue);
        break;
      default:
        ColorCalibrator.setFrequency(ColorCalibrator.Color.Other);
        break;
    }
  }
}
