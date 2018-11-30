package ca.mcgill.ecse211.tests;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.Game;
import ca.mcgill.ecse211.project.GameParameters;
import ca.mcgill.ecse211.project.GameUtil;
import ca.mcgill.ecse211.project.Navigation;
import ca.mcgill.ecse211.project.RingSearcher;
import lejos.hardware.Button;

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

  /**
   * This enumeration stores the possible test types that can be tested on our robot
   */
  public enum Type {
    Navigation, Localization, UltrasonicSensor, LightSensor, RingDetection
  }

  /**
   * This method executes a specific type of tests
   * 
   * @param type The type of tests to execute
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
          ComponentTest.ringMotorTest();
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
   * This method runs Navigation tests
   * 
   * @throws OdometerExceptions
   */
  public static void navigationTest() throws OdometerExceptions {
    Navigation nav = new Navigation(Game.leftMotor, Game.rightMotor);
    nav.travelToWithCorrection(4, 2, false);
    nav.travelToWithCorrection(0, 0, false);
    nav.travelToWithCorrection(4, 2, false);
    nav.travelToWithCorrection(0, 0, false);
    nav.travelToWithCorrection(4, 2, false);
    nav.travelToWithCorrection(0, 0, false);
    nav.travelToWithCorrection(4, 2, false);
    nav.travelToWithCorrection(0, 0, false);
    nav.travelToWithCorrection(4, 2, false);
    nav.travelToWithCorrection(0, 0, false);
  }

  /**
   * This method runs Tunnel tests
   * 
   * @throws Exception
   */
  public static void tunnelTest() throws Exception {
    Navigation navigation = new Navigation(Game.leftMotor, Game.rightMotor);
    GameUtil.searchingFinder =
        new GameUtil.PathFinder(GameParameters.Island_LL, GameParameters.Island_UR);
    GameUtil.startingFinder = new GameUtil.PathFinder(GameParameters.US_LL, GameParameters.US_UR);
    Odometer.getOdometer().setXYT(1, 7, 90);
    navigation.goThroughTunnel();
    navigation.goThroughTunnel();
  }

  /**
   * This method runs Localization tests
   * 
   * @throws Exception
   */
  public static void localizationTest() throws OdometerExceptions {
    Navigation navigation = new Navigation(Game.leftMotor, Game.rightMotor);
    UltrasonicLocalizer us = new UltrasonicLocalizer(navigation, Game.leftMotor, Game.rightMotor);
    LightLocalizer lgLoc = new LightLocalizer(navigation, Game.leftMotor, Game.rightMotor);
    us.localize(Button.ID_LEFT);
    lgLoc.localize(GameParameters.SC);
  }

  /**
   * This method runs UltrasonicSensor tests
   * 
   * @throws Exception
   */
  public static void ultrasonicSensorTest() {

  }

  /**
   * This method runs LightSensor tests
   * 
   * @throws Exception
   */
  public static void lightSensorTest() {

  }

  /**
   * This method runs RingDetection tests
   * 
   * @throws Exception
   */
  public static void ringMotorTest() throws OdometerExceptions {
    Game.INSTANCE.usPoller.setStart(false);
    final RingSearcher searcher = new RingSearcher(Game.sensorMotor, Game.rodMotor);
    Navigation navigation = new Navigation(Game.leftMotor, Game.rightMotor);
    GameUtil.searchingFinder =
        new GameUtil.PathFinder(GameParameters.Island_LL, GameParameters.Island_UR);
    GameUtil.startingFinder = new GameUtil.PathFinder(GameParameters.US_LL, GameParameters.US_UR);
    Odometer.getOdometer().setXYT(1, 1, 0);
    int[] tree = {2, 2};
    int[][] other = {{2, 1}, {3, 2}, {2, 3}, {1, 2}};
    for (int i = 0; i < 4; i++) {
      navigation.travelToWithCorrection(other[i][0], other[i][1], false);
      navigation.turn(-90);
      if (i != 3) {
        navigation.searchRingSet(searcher, true, true);
      } else {
        navigation.searchRingSet(searcher, true, false);
      }
    }
  }

  /**
   * This method tests black line detection
   * 
   * @throws Exception
   */
  public static void blackLineTest() throws OdometerExceptions {
    Navigation navi = new Navigation(Game.leftMotor, Game.rightMotor);
    GameUtil.searchingFinder =
        new GameUtil.PathFinder(GameParameters.Island_LL, GameParameters.Island_UR);
    int[] arr1 = {0, 0};
    int[] arr2 = {0, 0};
    GameUtil.startingFinder = new GameUtil.PathFinder(arr1, arr2);
    int count = 0;
    while (count < 85) {
      navi.moveOneTileWithCorrection(0);
      count++;
      if (count % 7 == 0) {
        navi.turn(90);
      }
    }
  }

  /**
   * This method tests that music can be played
   * 
   * @throws Exception
   */
  public static void musicTest() {
    GameUtil.playMusic();
  }
}
