package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.tests.ComponentTest;
import ca.mcgill.ecse211.threads.GyroPoller;
import ca.mcgill.ecse211.threads.LightPoller;
import ca.mcgill.ecse211.threads.RGBPoller;
import ca.mcgill.ecse211.threads.SensorData;
import ca.mcgill.ecse211.threads.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class implements the main starting point for the Search and Localize lab
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class Main {
  /**
   * Whether Enable test
   */
  public static boolean test = true;

  /**
   * Type of test
   */
  public static ComponentTest.Type cType = ComponentTest.Type.LightSensor;

  /**
   * This method is our main entry point - instantiate objects used and set up sensor.
   * 
   * @param args an array of arguments that can be passed in via commandline or otherwise.
   * @throws OdometerExceptions
   */
  public static void main(String[] args) {
    try {
      Game g = Game.getGame()
      g.preparation();
      if (test) {
        Button.waitForAnyPress(); // Record choice (left or right press)
        (new Thread(){
            public void run() {
              try {
                ComponentTest.RingMotorTest();
              } catch (OdometerExceptions e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
              }
            }
        }).start();
      } else {
        g.runGame();
      }
      Button.waitForAnyPress();
      System.exit(0);
    } catch (OdometerExceptions e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

  }
}
