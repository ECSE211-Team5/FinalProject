package ca.mcgill.ecse211.project;

import java.util.Arrays;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.project.GameParameters.DemoType;
import ca.mcgill.ecse211.threads.LightPoller;
import ca.mcgill.ecse211.threads.RGBPoller;
import ca.mcgill.ecse211.threads.SensorData;
import ca.mcgill.ecse211.threads.ThreadControl;
import ca.mcgill.ecse211.threads.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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

  /**
   * This enumeration stores the possible states that our robot can be in during a competition
   */
  public enum Status {
    Idle, Localized, AtTunnel, AtTree
  }

  /**
   * This variable stores the current state that our robot is in during a competition
   */
  private Status status = Status.Idle;

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
  public boolean ready(UltrasonicLocalizer us, LightLocalizer lgLoc) {
    boolean wasEventProcessed = false;

    Status aStatus = status;
    switch (aStatus) {
      case Idle:
        try {
          localizeAndReadData(us, lgLoc);
        } catch (OdometerExceptions e) {
          e.printStackTrace();
        }
        setStatus(Status.Localized);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
        break;
    }
    Sound.beep();
    Sound.beep();
    return wasEventProcessed;
  }

  /**
   * This method navigates our robot to the tunnel or search area
   * 
   * @return A boolean that denotes whether our state transition occurred
   */
  public boolean navigateToTunnel(Navigation navigation, RingSearcher searcher) {
    boolean wasEventProcessed = false;

    Status aStatus = status;
    switch (aStatus) {
      case Localized:
        navigateTunnel(navigation);
        setStatus(Status.AtTunnel);
        wasEventProcessed = true;
        break;
      case AtTree:
        navigateBackTunnel(navigation, searcher);
        setStatus(Status.AtTunnel);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
        break;
    }

    return wasEventProcessed;
  }

  /**
   * This method navigates our robot to the starting corner
   * 
   * @return A boolean that denotes whether our state transition occurred
   * @throws InterruptedException
   */
  public boolean navigateToStart(Navigation navigation, RingSearcher searcher) {
    boolean wasEventProcessed = false;

    Status aStatus = status;
    switch (aStatus) {
      case AtTunnel:
        navigateStart(navigation, searcher);
        setStatus(Status.Idle);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
        break;
    }

    return wasEventProcessed;
  }

  /**
   * This method navigates our robot to the tree and tries to find rings
   * 
   * @return A boolean that denotes whether our state transition occurred
   */
  public boolean navigateToAndSearcherTree(Navigation nav, RingSearcher searcher) {
    boolean wasEventProcessed = false;

    Status aStatus = status;
    switch (aStatus) {
      case AtTunnel:
        searchRing(nav, searcher);
        setStatus(Status.AtTree);
        wasEventProcessed = true;
        break;
      default:
        // Other states do respond to this event
        break;
    }

    return wasEventProcessed;
  }

  /**
   * This method sets the current state that our robot is in
   * 
   * @param newStatus The new state to set as our robot's current status
   */
  private synchronized void setStatus(Status newStatus) {
    status = newStatus;
  }

  /**
   * This variable stores a ThreadController instance that controls our RGB sensor
   */
  public ThreadControl rgbPoller;

  /**
   * This variable stores a ThreadController instance that controls our light sensor
   */
  private ThreadControl lightPoller;

  /**
   * This variable stores a ThreadController instance that controls our ultrasonic sensor
   */
  public ThreadControl usPoller;

  /**
   * This variable stores an EV3LargeRegulatedMotor object instance that allows control of the left
   * motor connected to port A
   */
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * This variable stores an EV3LargeRegulatedMotor object instance that allows control of the right
   * motor connected to port D
   */
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * This variable stores an EV3LargeRegulatedMotor object instance that allows control of the motor
   * on storage rod
   */
  public static final EV3LargeRegulatedMotor sensorMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

  /**
   * This variable stores an EV3LargeRegulatedMotor object instance that allows control of the motor
   * on the rod for collecting rings
   */
  public static final EV3LargeRegulatedMotor rodMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

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
  public static final double SEN_DIS = 4.2;

  /**
   * This method reads data from the WiFi class (using another thread)
   */
  public void readData() {
    WiFi.readData();
  }

  /**
   * This method localizes our robot to the starting position localize and read data at the same
   * time
   * 
   * @throws OdometerExceptions
   */
  private void localizeAndReadData(UltrasonicLocalizer us, LightLocalizer lgLoc)
      throws OdometerExceptions {
    this.rgbPoller.setStart(false);

    // read data at the same time as the localization
    readData();

    // perform localization
    us.localize(Button.ID_LEFT);
    this.usPoller.setStart(false);
    lgLoc.localize(GameParameters.SC);
    Sound.beep();
    int[] starting = GameParameters.SC;
    Odometer.getOdometer().setXYT(starting[0], starting[1], starting[2]);
  }

  /**
   * This method navigates our robot to and go through the tunnel (from safe area to search area)
   */
  private void navigateTunnel(Navigation nav) {
    try {
      nav.goThroughTunnel();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * This method navigates our robot to and go though the tunnel (from search area to safe area)
   */
  private void navigateBackTunnel(Navigation nav, RingSearcher searcher) {
    try {
      rgbPoller.setStart(false);
      nav.goThroughTunnel();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * This method makes our robot navigate back to the starting position (after in the safe area) and
   * then try to drop all rings
   */
  private static void navigateStart(Navigation nav, RingSearcher searcher) {
    double[] starting = {(GameParameters.SC[0]+GameParameters.SCUS[0])/2.0,(GameParameters.SC[1]+GameParameters.SCUS[1])/2.0};
    //nav.travelToWithCorrection(starting[0], starting[1], false);
    nav.travelTo(starting[0], starting[1], 600);
    nav.turnTo(
        Math.toDegrees(nav.calculateAngleTo(GameParameters.SCUS[0], GameParameters.SCUS[1])));
    for(int i = 0; i < 5; i++) {Sound.beep();}
    GameUtil.playMusic();
  }

  /**
   * This method search and retrieve for the ring set from each side, including navigation from the
   * exit of tunnel to the ring set
   */
  private void searchRing(Navigation navigation, RingSearcher searcher) {
    this.rgbPoller.setStart(true);
    int[] tree = GameParameters.TREE_US;
    int[][] treeSides = {{tree[0], tree[1] + 1}, {tree[0] - 1, tree[1]}, {tree[0], tree[1] - 1},
        {tree[0] + 1, tree[1]}};

    // Find the nearest point relative to the robot
    int startingIndex = GameUtil.findClosestPointToRobot(treeSides);
    searcher.prepareSearch();
    System.out.println("treeSides.length " + treeSides.length);
    // search the tree counterclockwise
    for (int i = 0; i < treeSides.length; i++) {
      int[] side = treeSides[(startingIndex + i) % treeSides.length];
      if (GameUtil.isSafe(side)) {
        navigation.travelToWithCorrection(side[0], side[1], false);
        if (i == 0) {
          Sound.twoBeeps();
          Sound.beep();
        }

        // turn facing the ring set
        navigation.turnTo(Math.toDegrees(navigation.calculateAngleTo(tree[0], tree[1])));
        boolean doCorrection = !GameUtil.isIslandBoundary(side);
        if (i != treeSides.length - 1) {
          navigation.searchRingSet(searcher, doCorrection, true);
        } else {
          navigation.searchRingSet(searcher, doCorrection, false);
        }
      }
    }
    searcher.resetSearch();
  }

  /**
   * This method performs all the object instantiations and preparations necessary to get our robot
   * to compete
   * 
   * @throws OdometerExceptions
   */
  public void preparation() throws OdometerExceptions {
    // Motor Objects, and Robot related parameters
    Port usPort = LocalEV3.get().getPort("S1");
    // initialize multiple light ports in main
    Port[] lgPorts = new Port[3];

    // Light sensor sensor stuff
    lgPorts[0] = LocalEV3.get().getPort("S2");
    lgPorts[1] = LocalEV3.get().getPort("S3");
    lgPorts[2] = LocalEV3.get().getPort("S4");
    EV3ColorSensor[] lgSensors = new EV3ColorSensor[3];
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

    // colour detection sensor
    SampleProvider frontLight[] = new SampleProvider[1];
    frontLight[0] = lgSensors[2].getRGBMode();

//     TextLCD lcd = LocalEV3.get().getTextLCD();
//     Display odometryDisplay = new Display(lcd);
//     

    // Start odometer and odometer display
    Thread odoThread = new Thread(odometer);
    odoThread.start();
//     Thread odoDisplayThread = new Thread(odometryDisplay);
//     odoDisplayThread.start();
    // Start ultrasonic and light sensors
    usPoller = new UltrasonicPoller(usDistance, usData, sensorData);
    Thread usThread = new Thread(usPoller);
    usThread.start();
    lightPoller = new LightPoller(backLight, new float[2][backLight[1].sampleSize()], sensorData);
    Thread lightThread = new Thread(lightPoller);
    lightThread.start();

    rgbPoller = new RGBPoller(frontLight, new float[1][frontLight[0].sampleSize()], sensorData);
    Thread rgbThread = new Thread(rgbPoller);

    rgbThread.start();
    Sound.beep();
  }

  /**
   * This method is called when the after the robot has been prepared and is ready to compete
   * 
   * @throws OdometerExceptions
   */
  public void runGame() throws OdometerExceptions {
    // Start localizing
    final Navigation navigation = new Navigation(leftMotor, rightMotor);
    final UltrasonicLocalizer usLoc = new UltrasonicLocalizer(navigation, leftMotor, rightMotor);
    final LightLocalizer lgLoc = new LightLocalizer(navigation, leftMotor, rightMotor);
    final RingSearcher searcher = new RingSearcher(sensorMotor, rodMotor);

    Button.waitForAnyPress(); // Wait for button press to start
    INSTANCE.ready(usLoc, lgLoc);
    System.out.println("Grid_UR " + Arrays.toString(GameParameters.Grid_UR));
    System.out.println("Grid_LL " + Arrays.toString(GameParameters.Grid_LL));

    // instantiate path finder
    GameUtil.searchingFinder =
        new GameUtil.PathFinder(GameParameters.Island_LL, GameParameters.Island_UR);
    GameUtil.startingFinder = new GameUtil.PathFinder(GameParameters.US_LL, GameParameters.US_UR);
    INSTANCE.navigateToTunnel(navigation, searcher);
    INSTANCE.navigateToAndSearcherTree(navigation, searcher);
    INSTANCE.navigateToTunnel(navigation, searcher);
    INSTANCE.navigateToStart(navigation, searcher);
  }
}
