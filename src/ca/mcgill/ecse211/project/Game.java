package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensors.LightPoller;
import ca.mcgill.ecse211.sensors.SensorData;
import ca.mcgill.ecse211.sensors.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
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
  private static ThreadControl rgbThread;
  private static ThreadControl lightThread;
  private static ThreadControl motorThread;
  private static ThreadControl usThread;

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
   * Motor object instance that allows control of the motor on storage rod
   */
  public static final EV3LargeRegulatedMotor storageMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  
  /**
   * Motor object instance taht allows control of the motor on the rod for collecting rings
   */
  public static final EV3MediumRegulatedMotor rodMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
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
  public static final double TRACK = 11.5;

  /**
   * The distance between light sensor and the center of the robot in cm
   */
  public static final double SEN_DIS = 4.4;
  
  private static boolean hasReadData;
  
  /**
   * Prepare for the game: starting thread, read all arguments
   * 
   * @throws OdometerExceptions
   */
  public static void preparation() throws OdometerExceptions {
    // Motor Objects, and Robot related parameters
    Port usPort = LocalEV3.get().getPort("S1");
    // initialize multiple light ports in main
    Port[] lgPorts = new Port[3];
    
    // Light sesnor sensor stuff
    lgPorts[0] = LocalEV3.get().getPort("S2");
    lgPorts[1] = LocalEV3.get().getPort("S3");
    EV3ColorSensor[] lgSensors = new EV3ColorSensor[2];
    for (int i = 0; i < lgSensors.length; i++) {
      lgSensors[i] = new EV3ColorSensor(lgPorts[i]);
    }

    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

    // Sensor Related Stuff
    SensorData sensorData = SensorData.getSensorData();

    // Ultrasonic sensor stuff
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(usPort);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    SampleProvider backLight[] = new SampleProvider[2];
    backLight[0] = lgSensors[0].getRedMode();
    backLight[1] = lgSensors[1].getRedMode();
    
    TextLCD lcd = LocalEV3.get().getTextLCD();
    Display odometryDisplay = new Display(lcd);
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
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, sensorData);
    usPoller.start();
    usThread = usPoller;
    LightPoller lgPoller = new LightPoller(backLight, new float[2][backLight[1].sampleSize()], sensorData);
    lgPoller.start();
    lightThread = lgPoller;
    
    //Thread fLgPoller1 = new RGBPoller(frontLight, new float[frontLight.sampleSize()], sensorData);
    //fLgPoller1.start();
//    Thread gPoller = new GyroPoller(gProvider, new float[gProvider.sampleSize()], sensorData);
//    gPoller.start();
  }

  /**
   * Read data from the wifi class (using another thread)
   */
  public synchronized static void readData() {
    WiFi wifi = new WiFi();
  }

  /**
   * This method contains main logic for the game plays
   * 
   * @throws OdometerExceptions
   */
  public synchronized static void runGame() throws OdometerExceptions {
    final int buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    // Start localizing
    final Navigation navigation = new Navigation(leftMotor, rightMotor);
    final UltrasonicLocalizer usLoc = new UltrasonicLocalizer(navigation, leftMotor, rightMotor);
    final LightLocalizer lgLoc = new LightLocalizer(navigation, leftMotor, rightMotor);
    final RingSearcher searcher = new RingSearcher(storageMotor, rodMotor);
    // spawn a new Thread to avoid localization from blocking
    (new Thread() {
      public void run() {
        // target color

        (new Thread() {
          public void run() {
            readData();
            hasReadData = true;
            notify();
          }
        }).start();
        usLoc.localize(buttonChoice);
        lgLoc.localize(GameParameters.SC);
          try {
            while(!hasReadData) wait();
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
      }
    }).start();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
