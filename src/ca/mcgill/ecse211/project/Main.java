package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.tests.ComponentTest;
import lejos.hardware.Button;

/**
 * This class implements the main starting point for our final project
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
   * This variable decides whether or not to enable our tests
   */
  public static boolean test = false;

  /**
   * This variable stores the type of test that we want to perform
   */
  // public static ComponentTest.Type testType = ComponentTest.Type.RingDetection;

  /**
   * This method is our main entry point - instantiate objects and halt until a button is pressed
   * 
   * @param args an array of arguments that can be passed in via command line or otherwise
   * @throws OdometerExceptions
   */
  public static void main(String[] args) {
    try {
      Game.INSTANCE.preparation(); // for brevity and less object instantiations
      if (test) {
        Button.waitForAnyPress(); // Wait for a button on the EV3 Brick to be pressed
        (new Thread() {
          public void run() {
            try {
              ComponentTest.navigationTest();
            } catch (Exception e) {
              e.printStackTrace();
            }
          }
        }).start();
      } else {
        Game.INSTANCE.runGame(); // for brevity and less object instantiations
      }
      Button.waitForAnyPress();
      System.exit(0);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
