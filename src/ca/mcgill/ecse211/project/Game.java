package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.threads.LightPoller;
import ca.mcgill.ecse211.threads.RingSearcher;
import ca.mcgill.ecse211.threads.SensorData;
import ca.mcgill.ecse211.threads.ThreadControl;
import ca.mcgill.ecse211.threads.UltrasonicPoller;
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
 * This singleton contains all the methods and structures necessary to start competing in a game
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public enum Game {
  INSTANCE;
  // ------------------------
  // MEMBER VARIABLES
  // ------------------------

  /**
   * This enumeration stores the possible states that our robot can be in during a competition
   */
  public enum Status {
    Idle, Localization, NavigationSafe, NavigationSearch, RingSearch
  }

  /**
   * This variable stores the current state that our robot is in during a competition
   */
  private static Status status = Status.Idle;

  // ------------------------
  // INTERFACE
  // ------------------------
  /**
   * This method gets a string representation of the status of our robot
   * 
   * @return A string of the status variable
   */
  public String getStatusFullName() {
    return status.toString();
  }

  /**
   * This method gets the current status of our robot
   * 
   * @return A Status enumeration value
   */
  public Status getStatus() {
    return status;
  }

  /**
   * This method performs localizes our robot
   * 
   * @return A boolean that denotes whether our state transition occurred
   */
  public static boolean ready() {
    boolean wasEventProcessed = false;

    switch (status) {
      case Idle:
        // line 5 "model.ump"
        localize();
        setStatus(Status.Localization);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
    }

    return wasEventProcessed;
  }

  /**
   * This method prepares our robot to navigate to a tunnel or search area
   * 
   * @return A boolean that denotes whether our state transition occurred
   */
  public static boolean localized() {
    boolean wasEventProcessed = false;

    switch (status) {
      case Localization:
        // line 8 "model.ump"
        navigate();
        setStatus(Status.NavigationSafe);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
    }

    return wasEventProcessed;
  }

  /**
   * This method navigates our robot to the tunnel or search area
   * 
   * @return A boolean that denotes whether our state transition occurred
   */
  public static boolean navigatedToTunnel() {
    boolean wasEventProcessed = false;

    switch (status) {
      case NavigationSafe:
        // line 11 "model.ump"
        navigate();
        setStatus(Status.NavigationSearch);
        wasEventProcessed = true;
        break;
      case NavigationSearch:
        // line 17 "model.ump"
        navigate();
        setStatus(Status.NavigationSafe);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
    }

    return wasEventProcessed;
  }

  /**
   * This method navigates our robot to the starting corner
   * 
   * @return A boolean that denotes whether our state transition occurred
   * @throws InterruptedException
   */
  public static boolean navigatedToStart() throws InterruptedException {
    boolean wasEventProcessed = false;

    switch (status) {
      case NavigationSafe:
        // line 12 "model.ump"
        //wait();
        setStatus(Status.Idle);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
    }

    return wasEventProcessed;
  }

  /**
   * This method navigates our robot to the tree and tries to find rings
   * 
   * @return A boolean that denotes whether our state transition occurred
   */
  public static boolean navigatedToTree() {
    boolean wasEventProcessed = false;

    switch (status) {
      case NavigationSearch:
        // line 16 "model.ump"
        searchRing();
        setStatus(Status.RingSearch);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
    }

    return wasEventProcessed;
  }

  /**
   * This method is called when a ring is found and obtains it
   * 
   * @return A boolean that denotes whether our state transition occurred
   */
  public static boolean ringFound() {
    boolean wasEventProcessed = false;

    switch (status) {
      case RingSearch:
        // line 20 "model.ump"
        navigate();
        setStatus(Status.NavigationSearch);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
    }

    return wasEventProcessed;
  }

  /**
   * This method is called when a ring is not found
   * 
   * @return A boolean that denotes whether our state transition occurred
   */
  public static boolean ringNotFound() {
    boolean wasEventProcessed = false;

    switch (status) {
      case RingSearch:
        // line 21 "model.ump"
        navigate();
        setStatus(Status.NavigationSearch);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
    }

    return wasEventProcessed;
  }

  /**
   * This method sets the current state that our robot is in
   * 
   * @param newStatus The new state to set as our robot's current status
   */
  private static void setStatus(Status newStatus) {
    status = newStatus;
  }

  /**
   * This variable stores a ThreadController instance that controls our RGB sensor
   */
  private static ThreadControl rgbPoller;

  /**
   * This variable stores a ThreadController instance that controls our light sensor
   */
  private static ThreadControl lightPoller;

  /**
   * This variable stores a ThreadController instance that controls our motors
   */
  private static ThreadControl motorControlThread;

  /**
   * This variable stores a ThreadController instance that controls our ultrasonic sensor
   */
  private static ThreadControl usPoller;

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
   * Motor object instance that allows control of the motor on the rod for collecting rings
   */
  public static final EV3MediumRegulatedMotor rodMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * This variable stores the length of a tile in cm
   */
  public static final double TILE = 30.48;

  /**
   * This variable stores the radius of our wheels in cm
   */
  public static final double WHEEL_RAD = 2.15;

  /**
   * This variable holds the track distance between the center of the wheels in cm (measured and
   * adjusted based on trial and error)
   */
  public static final double TRACK = 11.5;

  /**
   * This variable stores the distance between the light sensor and center of the robot in cm
   */
  public static final double SEN_DIS = 4.4;

  /**
   * This variable decides when GameParameter data has been successfully read over WiFi
   */
  private static boolean hasReadData;

  /**
   * Read data from the WiFi class (using another thread)
   */
  public synchronized static void readData() {
    WiFi wifi = new WiFi();
  }

  /**
   * This method localizes our robot to the starting position
   */
  private synchronized static void localize() {
    switch (status) {
      case Idle:
        localized();
        break;
      default:
        break;
    }
  }

  /**
   * This method navigates our robot to a desired location
   */
  private synchronized static void navigate() {
    switch (status) {
      case Localization:
        //go to start
        break;
      case NavigationSafe:
        navigatedToTunnel();
        break;
      case NavigationSearch:
        navigatedToTree();
        break;
      case RingSearch:
        searchRing();
        break;
      default:
        break;
    }
  }

  /**
   * This method makes our robot search for a ring
   */
  private synchronized static void searchRing() {
    switch (status) {
      case NavigationSearch:
        for(int i = 0; i < 4; i++) {
          navigatedToTree();
          if(ringFound()) {
            
          } else {
            ringNotFound();
          }
        }
        break;
      default:
        break;
    }
  }

  /**
   * This method performs all the object instantiations and preparations necessary to get our robot
   * to compete
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
    usPoller = new UltrasonicPoller(usDistance, usData, sensorData);
    Thread usThread = new Thread(usPoller);
    usThread.start();
    lightPoller = new LightPoller(backLight, new float[2][backLight[1].sampleSize()], sensorData);
    Thread lightThread = new Thread(lightPoller);
    lightThread.start();

    // Thread fLgPoller1 = new RGBPoller(frontLight, new float[frontLight.sampleSize()],
    // sensorData);
    // fLgPoller1.start();
    // Thread gPoller = new GyroPoller(gProvider, new float[gProvider.sampleSize()], sensorData);
    // gPoller.start();
  }

  /**
   * This method is called when the after the robot has been prepared and is ready to compete
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

        ready();
        localized();

        
        
        (new Thread() {
          public void run() {
            readData();
            hasReadData = true;
            notify();
          }
        }).start();
        try {
          while (!hasReadData)
            wait();
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
        usLoc.localize(buttonChoice);
        lgLoc.localize(GameParameters.SC);
        searcher.search();
        searcher.retrieveRing();
        // ug collision detection always on
        // navigate to start
      }
    }).start();
  }
}
