package ca.mcgill.ecse211.tests;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.Game;
import ca.mcgill.ecse211.project.GameParameters;
import ca.mcgill.ecse211.project.GameUtil;
import ca.mcgill.ecse211.project.Navigation;
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
    nav.travelToWithCorrection(4, 2, false);
    nav.travelToWithCorrection(0, 0, false);
  }
  
  public static void tunnelTest() throws Exception {
    Navigation navigation = new Navigation(Game.leftMotor, Game.rightMotor);
    GameUtil.searchingFinder = new GameUtil.PathFinder(GameParameters.Island_LL, GameParameters.Island_UR);
    GameUtil.startingFinder = new GameUtil.PathFinder(GameParameters.US_LL, GameParameters.US_UR);
    //UltrasonicLocalizer us = new UltrasonicLocalizer(navigation, Game.leftMotor, Game.rightMotor);
    //LightLocalizer lgLoc = new LightLocalizer(navigation, Game.leftMotor, Game.rightMotor);
    //us.localize(Button.ID_LEFT);
    //lgLoc.localize(GameParameters.SC);
    Odometer.getOdometer().setXYT(1, 1, 0);
    navigation.goThroughTunnel();
    navigation.travelToWithCorrection(4, 6, false);
    navigation.goThroughTunnel();
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
    Game.INSTANCE.usPoller.setStart(false);
    navigation.travelToWithCorrection(6, 4, false);
    Sound.twoBeeps();
    navigation.travelToWithCorrection(2, 3, false);
    Sound.twoBeeps();
    navigation.travelToWithCorrection(0, 0, false);
    Game.INSTANCE.usPoller.setStart(true);

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
