package ca.mcgill.ecse211.Test;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.Game;
import ca.mcgill.ecse211.project.Navigation;

public class ComponentTest {
  public enum Type {
    Navigation, Localization, UltrasonicSensor, LightSensor, RingDetection
  }

  /**
   * This method selects test for each individual components of the design
   * 
   * @param type
   */
  public static void runTest(Type type) {

  }

  /**
   * Test for Navigation
   * @throws OdometerExceptions 
   */
  public static void navigationTest() throws OdometerExceptions {
    Navigation nav = new Navigation(Game.leftMotor, Game.rightMotor);
    nav.travelTo(1, 1, false);
    nav.travelTo(2, 1, false);
    nav.travelTo(3, 2, false);
    nav.travelTo(4, 2, false);
    nav.travelTo(1, 5, false);
  }

  /**
   * Test for localizationTest
   * @throws OdometerExceptions 
   */
  public static void localizationTest() throws OdometerExceptions {
  
  }

  /**
   * Test for UltrasonicSensor
   */
  public static void ultrasonicSensorTest() {

  }

  /**
   * Test for lightSensor
   */
  public static void lightSensorTest() {

  }

  /**
   * Test for ring detection
   */
  public static void RingDetection() {

  }

}
