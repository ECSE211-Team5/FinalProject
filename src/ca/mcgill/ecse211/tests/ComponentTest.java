package ca.mcgill.ecse211.tests;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.Game;
import ca.mcgill.ecse211.project.Navigation;
import ca.mcgill.ecse211.threads.RingSearcher;

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
    nav.travelTo(4, 2, false);
    nav.travelTo(0, 0, false);
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
   * @throws OdometerExceptions 
   */
  public static void RingMotorTest() throws OdometerExceptions {
    final RingSearcher searcher = new RingSearcher(Game.storageMotor, Game.rodMotor);
    searcher.setStart(true);
    Thread t = new Thread(searcher);
    t.start();
    try {
      Thread.sleep(20000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    searcher.setStart(false);
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    searcher.setStart(true);
  }

}
