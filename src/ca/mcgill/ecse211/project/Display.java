package ca.mcgill.ecse211.project;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.threads.SensorData;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta) on an EV3 brick
 * display
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class Display implements Runnable {
  private SensorData sensdata;
  private Odometer odo;
  private TextLCD lcd;
  private double[] position;
  private double[] data;
  private int[] rgb;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * This is the class constructor for the Display class
   * 
   * @param lcd A TextLCD object instance to control
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd) throws OdometerExceptions {
    this.odo = Odometer.getOdometer();
    this.sensdata = SensorData.getSensorData();
    this.lcd = lcd;
  }

  /**
   * This is the overloaded class constructor for a display object
   * 
   * @param lcd A TextLCD object instance to control
   * @param timeout A duration of time to update the display for
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  /**
   * This method is called when the Display thread is started.
   */
  public void run() {
    lcd.clear();

    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();

    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();
      rgb = sensdata.getRGB();

      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      // The last two parameters to lcd.drawString denote the x and y coordinate to draw at.
      // lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      // lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      // lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      // lcd.drawString("LL: " + numberFormat.format(sensdata.getL()[0]), 0, 3);
      // lcd.drawString("LR: " + numberFormat.format(sensdata.getL()[1]), 0, 4);
      // lcd.drawString("D: " + numberFormat.format(sensdata.getD()), 0, 5);

      lcd.drawString(String.format("(R: %d G: %d B: %d)", (int) rgb[0], (int) rgb[1], (int) rgb[2]),
          0, 4);
      // if (ColorCalibrator.getColor((int) rgb[0], (int) rgb[1],
      // (int) rgb[2]) != ColorCalibrator.Color.Other) {
      // lcd.drawString("Object Detected", 0, 5);
      // } else {
      // // Draw whitespace on our display
      // lcd.drawString(" ", 0, 5);
      // }

      lcd.drawString(String.format("%1$-10s", ColorCalibrator.getColor().toString()), 0, 6);
      lcd.drawString("A:" + numberFormat.format(Game.sensorMotor.getTachoCount()), 0, 7);

      // lcd.drawString(String.format("(r: %d", (int)rgb[0]), 0, 3);
      // lcd.drawString(String.format("(g: %d", (int)rgb[1]), 0, 4);
      // lcd.drawString(String.format("(b: %d", (int)rgb[2]), 0, 5);

      // This ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);
  }
}
