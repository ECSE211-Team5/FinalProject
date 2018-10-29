package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensors.GyroPoller;
import ca.mcgill.ecse211.sensors.LightPoller;
import ca.mcgill.ecse211.sensors.RGBPoller;
import ca.mcgill.ecse211.sensors.SensorData;
import ca.mcgill.ecse211.sensors.UltrasonicPoller;
import lejos.hardware.Button;
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
public class Game {
  // Motor Objects, and Robot related parameters
  //private static final Port usPort = LocalEV3.get().getPort("S1");
  // initialize multiple light ports in main
  private static Port[] lgPorts = new Port[3];
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
//  private static final Port gPort = LocalEV3.get().getPort("S4");
//  private static EV3GyroSensor gSensor = new EV3GyroSensor(gPort);

  /**
   * Motor object instance that allows control of the left motor connected to port A
   */
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * Motor object instance that allows control of the right motor connected to port D
   */
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  /**
   * length of the tile
   */
  public static final double TILE = 30.48;

  /**
   * This variable denotes the radius of our wheels in cm.
   */
  public static final double WHEEL_RAD = 2.15;

  /**
   * This variable denotes the track distance between the center of the wheels in cm (measured and
   * adjusted based on trial and error).
   */
  public static double TRACK = 11.5;

  public static double SEN_DIS = 4.4;
  /**
   * Prepare for the game: starting thread, read all arguments
   * 
   * @throws OdometerExceptions
   */
  public static void preparation() throws OdometerExceptions {
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    Display odometryDisplay = new Display(lcd);

    // Sensor Related Stuff
    SensorData sensorData = SensorData.getSensorData();

    // Ultrasonic sensor stuff
    //@SuppressWarnings("resource")
    //SensorModes usSensor = new EV3UltrasonicSensor(usPort);
    //SampleProvider usDistance = usSensor.getMode("Distance");
    //float[] usData = new float[usDistance.sampleSize()];

    // Light sesnor sensor stuff
    // @SuppressWarnings("resource")
    lgPorts[0] = LocalEV3.get().getPort("S2");
    lgPorts[1] = LocalEV3.get().getPort("S3");
    //lgPorts[2] = LocalEV3.get().getPort("S4");
    // lgPorts[2] = LocalEV3.get().getPort("S4");
    EV3ColorSensor[] lgSensors = new EV3ColorSensor[2];
    for (int i = 0; i < lgSensors.length; i++) {
      lgSensors[i] = new EV3ColorSensor(lgPorts[i]);
    }

    SampleProvider backLight1 = lgSensors[0].getRedMode();
    SampleProvider backLight2 = lgSensors[1].getRedMode();
    //SampleProvider frontLight = lgSensors[2].getRGBMode();

    // SampleProvider frontLight2 = lgSensors[2].getRedMode();

    //SampleProvider gProvider = gSensor.getAngleMode();

    // STEP 1: LOCALIZE to (1,1)
    // ButtonChoice left or right
    lcd.clear();
    lcd.drawString("<  Left  |  Right >", 0, 0);
    lcd.drawString(" falling | rising  ", 0, 1);
    lcd.drawString("  edge   |  edge   ", 0, 2);
    lcd.drawString("        \\/        ", 0, 3);
    lcd.drawString("  Color Detection  ", 0, 4);

    // Start odometer and odometer display
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();

    // Start ultrasonic and light sensors
    //Thread usPoller = new UltrasonicPoller(usDistance, usData, sensorData);
    //usPoller.start();
    Thread bLgPoller1 = new LightPoller(backLight1, new float[backLight1.sampleSize()], sensorData);
    bLgPoller1.start();
    Thread bLgPoller2 = new LightPoller(backLight2, new float[backLight2.sampleSize()], sensorData);
    bLgPoller2.start();
    //Thread fLgPoller1 = new RGBPoller(frontLight, new float[frontLight.sampleSize()], sensorData);
    //fLgPoller1.start();
//    Thread gPoller = new GyroPoller(gProvider, new float[gProvider.sampleSize()], sensorData);
//    gPoller.start();
  }

  /**
   * perform localization
   */
  public static void localization() {

  }

  /**
   * Read data from the wifi class (using another thread)
   */
  public static void readData() {
    WiFi wifi = new WiFi();
  }

  /**
   * This method contains main logic for the game plays
   * 
   * @throws OdometerExceptions
   */
  public static void runGame() throws OdometerExceptions {
    final int buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    // Start localizing
    final Navigation navigation = new Navigation(leftMotor, rightMotor);
    final UltrasonicLocalizer usLoc = new UltrasonicLocalizer(navigation, leftMotor, rightMotor);
    final LightLocalizer lgLoc = new LightLocalizer(navigation, leftMotor, rightMotor);
    final RingSearcher searcher = new RingSearcher(navigation, leftMotor, rightMotor);
    // spawn a new Thread to avoid localization from blocking
    (new Thread() {
      public void run() {
        // target color

        usLoc.localize(buttonChoice);
        // lgLoc.localize(SC);
        // gSensor.reset();
        // // TODO: delete test code
        // try {
        // Odometer odometer = Odometer.getOdometer();
        // // STEP 2: MOVE TO START OF SEARCH AREA
        // navigation.travelTo(LLx, LLy, false);
        // Sound.beep();
        // // STEP 3: SEARCH ALL COORDINATES
        // navigation.travelTo(odometer.getXYT()[0], URy+0.5, true);
        // navigation.travelTo(URx, URy+0.5, true);
        // navigation.travelTo(URx, URy, false);
        // // STEP 4: NAVIGATE TO URx, URy
        // } catch (OdometerExceptions e) {
        // // TODO Auto-generated catch block
        // e.printStackTrace();
        // }

      }
    }).start();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
