package ca.mcgill.ecse211.tests;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.Game;
import ca.mcgill.ecse211.project.GameParameters;
import ca.mcgill.ecse211.project.Navigation;
import ca.mcgill.ecse211.threads.RingSearcher;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * This singleton contains all the methods and structures necessary to test our robot and its
 * components
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public enum ComponentTest {
  INSTANCE;

  public enum Type {
    Navigation, Localization, UltrasonicSensor, LightSensor, RingDetection
  }

  /**
   * This method selects test for each individual components of the design
   * 
   * @param type
   * @throws Exception
   */
  public static void runTest(Type testType) throws Exception {
    try {
      switch (testType) {
        case Navigation:
          ComponentTest.navigationTest();
          break;
        case Localization:
          ComponentTest.localizationTest();
          break;
        case UltrasonicSensor:
          ComponentTest.ultrasonicSensorTest();
          break;
        case LightSensor:
          ComponentTest.lightSensorTest();
          break;
        case RingDetection:
          //ComponentTest.ringMotorTest();
          break;
        default:
          System.out.println("Invalid test type selected");
          break;
      }
    } catch (Exception e) {
      throw e;
    }
  }

  /**
   * Test for Navigation
   * 
   * @throws OdometerExceptions
   */
  public static void navigationTest() throws OdometerExceptions {
    Navigation nav = new Navigation(Game.leftMotor, Game.rightMotor);
    nav.travelTo(4, 2, false);
    nav.travelTo(0, 0, false);
  }

  /**
   * Test for localizationTest
   * 
   * @throws OdometerExceptions
   */
  public static void localizationTest() throws OdometerExceptions {
    Navigation navigation = new Navigation(Game.leftMotor, Game.rightMotor);
    UltrasonicLocalizer us = new UltrasonicLocalizer(navigation, Game.leftMotor, Game.rightMotor);
    LightLocalizer lgLoc = new LightLocalizer(navigation, Game.leftMotor, Game.rightMotor);
    us.localize(Button.ID_LEFT);
    lgLoc.localize(GameParameters.SC);
    Game.usPoller.setStart(false);
    navigation.travelTo(6, 4, false);
    Sound.twoBeeps();
    navigation.travelTo(2, 3, false);
    Sound.twoBeeps();
    navigation.travelTo(0, 0, false);
    Game.usPoller.setStart(true);

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
   * 
   * @throws OdometerExceptions
   */
//  public static void ringMotorTest() throws OdometerExceptions {
//    final RingSearcher searcher = new RingSearcher(Game.storageMotor, Game.rodMotor);
//    searcher.setStart(true);
//    Thread t = new Thread(searcher);
//    t.start();
//    try {
//      Thread.sleep(20000);
//    } catch (InterruptedException e) {
//      e.printStackTrace();
//    }
//    searcher.setStart(false);
//    try {
//      Thread.sleep(2000);
//    } catch (InterruptedException e) {
//      e.printStackTrace();
//    }
//    searcher.setStart(true);
//  }
}
