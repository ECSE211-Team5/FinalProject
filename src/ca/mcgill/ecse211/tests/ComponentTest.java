package ca.mcgill.ecse211.tests;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.Game;
import ca.mcgill.ecse211.project.GameParameters;
import ca.mcgill.ecse211.project.GameUtil;
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
    Odometer.getOdometer().setXYT(1, 7, 90);
    navigation.goThroughTunnel();
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
  public static void ringMotorTest() throws OdometerExceptions {
    final RingSearcher searcher = new RingSearcher(Game.sensorMotor, Game.rodMotor);
    Navigation navigation = new Navigation(Game.leftMotor, Game.rightMotor);
    GameUtil.searchingFinder = new GameUtil.PathFinder(GameParameters.Island_LL, GameParameters.Island_UR);
    GameUtil.startingFinder = new GameUtil.PathFinder(GameParameters.US_LL, GameParameters.US_UR);
    Odometer.getOdometer().setXYT(1, 1, 0);
    int[] tree = {2,2};
    int[][] other = {{2,1}, {3,2}, {2,3}, {1,2}};
    for(int i = 0; i < 4; i++) {
      navigation.travelToWithCorrection(other[i][0], other[i][1],false);
      navigation.turn(-90);
      navigation.approachRingSet(searcher);
    }
  }
}
