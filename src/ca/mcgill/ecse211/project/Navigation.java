package ca.mcgill.ecse211.project;

import java.util.ArrayList;
import java.util.Collections;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.threads.SensorData;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The Navigator class extends the functionality of the Navigation class. It offers an alternative
 * travelTo() method which uses a state machine to implement obstacle avoidance.
 * 
 * The Navigator class does not override any of the methods in Navigation. All methods with the same
 * name are overloaded i.e. the Navigator version takes different parameters than the Navigation
 * version.
 * 
 * This is useful if, for instance, you want to force travel without obstacle detection over small
 * distances. One place where you might want to do this is in the ObstacleAvoidance class. Another
 * place is methods that implement specific features for future milestones such as retrieving an
 * object.
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class Navigation {
  private static final int RIGHT_MOTOR_RING_COR = -50;
  private static final int LEFT_MOTOR_RING_COR = -40;
  private static final int FORWARD_SPEED = 250;
  private static final int ROTATE_SPEED = 250;
  private static final int TUNNEL_SPEED = 600;
  private static final int TUNNEL_CORRECTION = -3;
  private static final int Q_ACCELERATION = 3000;
  private static final int N_ACCELERATION = 300;

  private static int leftBlackLineThre = -10;
  private static int rightBlackLineThre = -10;

  // angles for detecting if one sensor misses the black lin
  private static final int ZERO_DEGREE_LOW = 350;
  private static final int ZERO_DEGREE_HIGH = 10;
  private static final int ZERO_AFTER_LOW = 335;
  private static final int ZERO_AFTER_HIGH = 25;
  private static final int OTHER_ANGLE_TOLERENCE = 25;

  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private Odometer odometer;
  private SensorData data;

  /**
   * This navigation class constructor sets up our robot to begin navigating a particular map
   * 
   * @param leftMotor An EV3LargeRegularedMotor object instance that allows control of the left
   *        motor
   * @param rightMotor An EV3LargeRegularedMotor object instance that allows control of the right
   *        motor
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor)
      throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.data = SensorData.getSensorData();
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {this.leftMotor,
        this.rightMotor}) {
      motor.stop();
      motor.setAcceleration(Q_ACCELERATION);
    }
  }

  /**
   * This method sets the motor acceleration speed to a lower threshold value
   */
  public void setSlowAcc() {
    leftMotor.setAcceleration(N_ACCELERATION);
    rightMotor.setAcceleration(N_ACCELERATION);
  }

  /**
   * This method calculates the angle necessary for the robot to rotate to face a certain point
   * 
   * @param x The x coordinate of the point to face
   * @param y The y coordinate of the point to face
   * @return The rotation angle in degrees
   */
  public double calculateAngleTo(double x, double y) {
    double dX = x - odometer.getXYT()[0];
    double dY = y - odometer.getXYT()[1];
    double theta = Math.atan(dX / dY);
    if (dY < 0 && theta < Math.PI)
      theta += Math.PI;
    return theta;
  }

  /**
   * This method makes our robot travel to a point by first rotating it and then traversing to the
   * point
   * 
   * @param x The x coordinate of the point to travel to
   * @param y The y coordinate of the point to travel to
   */
  public void travelTo(double x, double y, int speed) {
    double dX = x - odometer.getXYT()[0];
    double dY = y - odometer.getXYT()[1];
    double theta = calculateAngleTo(x, y);

    // Euclidean distance calculation.
    double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));

    turnTo(Math.toDegrees(theta));
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }


    leftMotor.rotate(convertDistance(Game.WHEEL_RAD, distance * Game.TILE), true);
    rightMotor.rotate(convertDistance(Game.WHEEL_RAD, distance * Game.TILE), false);

  }

  /**
   * This method makes the robot travel to a desired position by checking its orientation and
   * corrects it if necessary to align with the lines on the grid.
   * 
   * @param x The x coordinate to travel to
   * @param y The y coordinate to travel to
   * @param avoid This boolean decides if the robot will pay attention to readings from the
   *        ultrasonic sensor to avoid obstacles when navigating
   */
  public void travelToWithCorrection(int x, int y, boolean avoid) {
    int px = (int) Math.round(odometer.getXYT()[0]);
    int py = (int) Math.round(odometer.getXYT()[1]);
    int[] cur = {px, py};
    int[] destination = {x, y};
    ArrayList<Character> instruction = new ArrayList<Character>();

    // use path finder to find path based on the different areas the robot is in
    // OUT: instruction: contains a list of instruction for the robot to move to the destination
    if (GameParameters.getType(px, py) == GameParameters.AreaType.InStarting) {
      GameUtil.startingFinder.tryFindPath(cur, destination, instruction);
    } else {
      GameUtil.searchingFinder.tryFindPath(cur, destination, instruction);
    }

    // use the instruction modified by the pathFind to move to the destination
    char lastStep = ' ';
    int theta = 0;

    while (instruction.size() > 0) {
      char step = instruction.remove(instruction.size() - 1);
      // if the step is different from the last one, rotate to corresponding rotation
      if (step != lastStep) {
        theta = charToRotation(step);
        turnTo(theta);
      }

      // add a value to the robot traveled distance
      if (step == GameUtil.leftInstruction) {
        px--;
      } else if (step == GameUtil.rightInstruction) {
        px++;
      } else if (step == GameUtil.upInstruction) {
        py++;
      } else {
        py--;
      }
      lastStep = step;

      moveWithCorrection(1, theta);
      // get the position of the robot
      double[] position = odometer.getXYT();
      if (Math.round(position[0]) == px && Math.round(position[1]) == py) {
        // this means that the robot is at the point, so set the position to the point
        odometer.setX(px);
        odometer.setY(py);
      } else {
        // otherwise some problem might happened and we are not at the desired point, push the
        // instruction back
        instruction.add(step);
        // reset the added value to last point
        if (step == GameUtil.leftInstruction) {
          px++;
        } else if (step == GameUtil.rightInstruction) {
          px--;
        } else if (step == GameUtil.upInstruction) {
          py--;
        } else {
          py++;
        }
      }
    }
  }

  /**
   * This method converts directions our robot can face (up, down, right, left) into degrees
   * 
   * @param direction The direction to convert
   * @return The angle the robot should face in degrees
   */
  private int charToRotation(char direction) {
    if (direction == GameUtil.upInstruction) {
      return 0;
    } else if (direction == GameUtil.downInstruction) {
      return 180;
    } else if (direction == GameUtil.leftInstruction) {
      return 270;
    } else if (direction == GameUtil.rightInstruction) {
      return 90;
    } else {
      return 0;
    }
  }

  /**
   * This method moves our robot a certain distance with corrections if required.
   * 
   * @param distance The distance our robot is to travel
   * @param theta The angle to be corrected to upon crossing a tile
   */
  public synchronized void moveWithCorrection(double distance, double theta) {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // correct error of the distance
    int tiles = Math.abs((int) Math.round(distance));
    for (int i = 0; i < tiles; i++) {
      moveOneTileWithCorrection(theta);
    }
  }

  /**
   * This method moves the robot backwards until a black line is detected
   */
  private synchronized void moveBackWithCorrection() {
    // leftMotor.setAcceleration(N_ACCELERATION);
    // rightMotor.setAcceleration(N_ACCELERATION);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.backward();
    rightMotor.backward();
    moveUntilLineDetection(true);
  }

  /**
   * This method moves the robot one tile until it detects a black line
   * 
   * @param theta The angle to be corrected to upon crossing a tile
   */
  public void moveOneTileWithCorrection(double theta) {
    // leftMotor.setAcceleration(N_ACCELERATION);
    // rightMotor.setAcceleration(N_ACCELERATION);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    moveUntilLineDetection(true);
    odometer.setTheta(theta);
  }

  /**
   * This method moves the robot until a black line has been detected
   * 
   * @param checkForBlackLine This boolean decides whether to check for a black line.
   */
  private void moveUntilLineDetection(boolean checkForBlackLine) {
    double rotation = 0;
    int lastTachoCount = 0;
    char detacted = ' ';

    while (leftMotor.isMoving() || rightMotor.isMoving()) {
      // get left and right sensor values
      double left = data.getL()[0];
      double right = data.getL()[1];

      // Once left / right sensor has detect a black line, record the correct robot rotation and
      // the tacho count for the other sensor(motor), in case if the other sensor miss the black
      // line
      if (left < leftBlackLineThre) {
        leftMotor.setAcceleration(Q_ACCELERATION);
        rightMotor.setAcceleration(Q_ACCELERATION);
        leftMotor.stop(true);
        if (detacted == ' ') {
          detacted = 'l';
          rotation = odometer.getXYT()[2];
          lastTachoCount = rightMotor.getTachoCount();
        }
      }
      if (right < rightBlackLineThre) {
        leftMotor.setAcceleration(Q_ACCELERATION);
        rightMotor.setAcceleration(Q_ACCELERATION);
        rightMotor.stop(true);
        if (detacted == ' ') {
          detacted = 'r';
          rotation = odometer.getXYT()[2];
          lastTachoCount = leftMotor.getTachoCount();
        }
      }
    }
    double afterRotation = odometer.getXYT()[2];
    boolean missedBlackLine = false;

    // if the robot is rotated more than 20 degree (more than what we expected to correct, then it
    // means
    // one sensor has missed a black line), do correction again
    if (rotation < ZERO_DEGREE_HIGH || rotation > ZERO_DEGREE_LOW) {
      missedBlackLine = !(afterRotation > ZERO_AFTER_LOW || afterRotation < ZERO_AFTER_HIGH);
    } else {
      missedBlackLine = Math.abs(afterRotation - rotation) > OTHER_ANGLE_TOLERENCE;
    }

    // if one black line is missed, rotate the robot back to before blackline correction
    if (missedBlackLine && checkForBlackLine) {
      leftMotor.setSpeed(leftMotor.getMaxSpeed());
      rightMotor.setSpeed(rightMotor.getMaxSpeed());
      if (detacted == 'l')
        rightMotor.rotate(-(rightMotor.getTachoCount() - lastTachoCount));
      else if (detacted == 'r')
        leftMotor.rotate(-(leftMotor.getTachoCount() - lastTachoCount));
    }

    // rotate one sensor distance to make the sensor off the black line
    leftMotor.setAcceleration(N_ACCELERATION);
    rightMotor.setAcceleration(N_ACCELERATION);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(Game.WHEEL_RAD, Game.SEN_DIS), true);
    rightMotor.rotate(convertDistance(Game.WHEEL_RAD, Game.SEN_DIS + 0.5), false);

    // go backward to correct again
    if (missedBlackLine && checkForBlackLine) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.backward();
      rightMotor.backward();
      moveUntilLineDetection(true);
    }
  }

  /**
   * This method turns our robot to a desired angle
   * 
   * @param angle The angle we want our robot to turn to (in degrees)
   */
  public synchronized void turnTo(double angle) {
    double dTheta;

    dTheta = angle - odometer.getXYT()[2];
    if (dTheta < 0)
      dTheta += 360;

    // TURN RIGHT
    if (dTheta > 180) {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      leftMotor.rotate(-convertAngle(Game.WHEEL_RAD, Game.TRACK, 360 - dTheta), true);
      rightMotor.rotate(convertAngle(Game.WHEEL_RAD, Game.TRACK, 360 - dTheta), false);
    }
    // TURN LEFT
    else {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      leftMotor.rotate(convertAngle(Game.WHEEL_RAD, Game.TRACK, dTheta), true);
      rightMotor.rotate(-convertAngle(Game.WHEEL_RAD, Game.TRACK, dTheta), false);
    }
  }

  /**
   * This method moves our robot through a tunnel by first finding the tunnel based on the lower
   * left and upper right coordinates provided in GameParameters, aligning itself with the tunnel
   * and then finally moving through it. the tunnel
   * 
   * @throws Exception
   */
  public void goThroughTunnel() throws Exception {
    int distance = 0;
    int[] ll, ur;
    // first use ll and ur coordinate to calculate lr and ul of the tunnel
    ll = GameParameters.TN_LL;
    ur = GameParameters.TN_UR;
    int[] lr = {ll[0], ur[1]};
    int[] ul = {ur[0], ll[1]};

    // clone the four points (to make sure we are not modifying the original one)
    int[][] corners = {ll.clone(), lr.clone(), ul.clone(), ur.clone()};
    ArrayList<int[]> notIn = new ArrayList<int[]>();
    ArrayList<int[]> points = new ArrayList<int[]>();
    double[] position = odometer.getXYT();

    // search for the points that are the same as the current area of the robot
    // these are the entrance of the tunnel, also find the other two points, those
    // are the exit of the tunnel
    GameParameters.AreaType type =
        GameParameters.getType((int) Math.round(position[0]), (int) Math.round(position[1]));
    for (int[] point : corners) {
      if (GameParameters.getType(point[0], point[1]) == type) {
        points.add(point);
      } else {
        notIn.add(point);
      }
    }

    // Sort the two point at exit by the distance to the destination
    if (type == GameParameters.AreaType.InStarting) {
      Collections.sort(notIn, new GameUtil.RingSetComparator());
    } else if (type == GameParameters.AreaType.Searching) {
      Collections.sort(notIn, new GameUtil.StartingComparator());
    }

    // find the direction and length of the tunnel
    // we know the entrance two points of the tunnel, so this means
    // the two points must have either x or y coordinate identical.
    // that's the direction of the tunnel as well
    // after identify it's direction, we find whether it is positive
    // or negative directed
    if (points.get(0)[0] == points.get(1)[0]) {
      distance = Math.abs(notIn.get(0)[0] - points.get(0)[0]);
      int multi = notIn.get(0)[0] - points.get(0)[0] < 0 ? 1 : -1;
      travelToTunnelEntrance(points, 0, multi);
      for (int i = 0; i < notIn.size(); i++) {
        // this step is to find the nearest two points that we can go two
        // after exit the tunnel
        notIn.get(i)[0] = notIn.get(i)[0] - multi * 1;
      }
    } else {
      distance = Math.abs(notIn.get(0)[1] - points.get(0)[1]);
      int multi = notIn.get(0)[1] - points.get(0)[1] < 0 ? 1 : -1;
      travelToTunnelEntrance(points, 1, multi);
      for (int i = 0; i < notIn.size(); i++) {
        // this step is to find the nearest two points that we can go two
        // after exit the tunnel
        notIn.get(i)[1] = notIn.get(i)[1] - multi * 1;
      }
    }

    double[] tunnelEnd = GameUtil.average(notIn.get(0), notIn.get(1));
    double angleThoughTunnel = Math.toDegrees(calculateAngleTo(tunnelEnd[0], tunnelEnd[1]));
    turnTo(angleThoughTunnel);

    // goback To correct
    moveBackWithCorrection();

    // turn left -5 to correct the effect of the weight
    forward(TUNNEL_SPEED, 0.5);
    turn(TUNNEL_CORRECTION);
    if (distance == 1) {
      forward(TUNNEL_SPEED, distance + 1);
    } else {

      forward(TUNNEL_SPEED, distance + 1);
    }

    odometer.setTheta(angleThoughTunnel);
    // leftMotor.setAcceleration(N_ACCELERATION);
    // rightMotor.setAcceleration(N_ACCELERATION);
    // // rotate additional sensor distances to make sure the sensor will not on the balck line
    // leftMotor.rotate(convertDistance(Game.WHEEL_RAD, 2*Game.SEN_DIS), true);
    // rightMotor.rotate(convertDistance(Game.WHEEL_RAD, 2*Game.SEN_DIS), false);
    this.moveOneTileWithCorrection(angleThoughTunnel);
    double[] after = GameUtil.average(notIn.get(0), notIn.get(1));
    odometer.setX(after[0]);
    odometer.setY(after[1]);
    // go to the nearest safe point near tunnel
    for (int[] p : notIn) {
      if (GameUtil.isSafe(p)) {
        double toPointAngle = Math.toDegrees(calculateAngleTo(p[0], p[1]));
        turnTo(toPointAngle);
        this.moveOneTileWithCorrection(toPointAngle);
        odometer.setX(p[0]);
        odometer.setY(p[1]);
        break;
      }
    }
  }

  /**
   * This method navigates our robot to the entrance of a tunnel
   * 
   * @param points An ArrayList of points to navigate our robot to the tunnel entrance
   * @param n An integer index to modify the list of points at
   * @param multiplier An integer to add onto each point
   */
  private void travelToTunnelEntrance(ArrayList<int[]> points, int n, int multiplier) {
    int[] safePoint = new int[2];
    for (int i = 0; i < points.size(); i++) {
      points.get(i)[n] = points.get(i)[n] + multiplier * 1;
    }

    // sort the entrance by the distance to the robot so that we can go to the closer one
    Collections.sort(points, new GameUtil.RobotComparator());
    // find the first safe point
    for (int[] p : points) {
      if (GameUtil.isSafe(p)) {
        safePoint = p;
        break;
      }
    }

    int[] beforePoint = safePoint;

    // find the nearst points to the entrance of the tunnel
    double[] center = GameUtil.average(points.get(0), points.get(1));

    // travel to the point
    this.travelToWithCorrection(beforePoint[0], beforePoint[1], false);
    // leftMotor.setAcceleration(N_ACCELERATION);
    // rightMotor.setAcceleration(N_ACCELERATION);
    travelTo(center[0], center[1], FORWARD_SPEED);
  }

  /**
   * This method moves our robot towards a ring set by noting ultrasonic sensor readings
   * 
   * @param searcher A RingSearcher object instance to detect ring colors and navigate around a ring
   *        set
   * @param correct A boolean that decides whether to correct our position when searching for a ring
   * @param reset A boolean that decides whether to rotate the rod motor to its original position
   */
  public void searchRingSet(RingSearcher searcher, boolean correct, boolean reset) {
    // Go backward to detect the line and correct the rotation
    // leftMotor.setAcceleration(N_ACCELERATION);
    // rightMotor.setAcceleration(N_ACCELERATION);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    double theta = odometer.getXYT()[2];

    // if we do correction, we need to forward more (for the sensor distance)
    if (correct) {
      leftMotor.backward();
      rightMotor.backward();
      moveUntilLineDetection(true);
      // Forward for 3 cm (approach the ring set)
      // forward(FORWARD_SPEED, 2.5 / Game.TILE);
    } else {
      // forward(FORWARD_SPEED, 2 / Game.TILE);
    }
    searcher.prepareRetrieve();
    // rotate a little to the left to make sure that the sensor can detect the ring
    // detect the ring color and beep based on the color
    searcher.search(-165);
    if (correct) {
      forward(FORWARD_SPEED, 2.8 / Game.TILE);
    } else {
      forward(FORWARD_SPEED, 3.8 / Game.TILE);
    }
    searcher.detectColor();
    searcher.search(-190);
    searcher.detectColor();

    // rotate back
    // leftMotor.rotate(-LEFT_MOTOR_RING_COR, false);
    // prepare for retrieving the ring
    searcher.finishSearch();

    rightMotor.rotate(-40, false);
    searcher.safeRod();
    if (correct) {
      forward(FORWARD_SPEED, 3.7 / Game.TILE);
    } else {
      forward(FORWARD_SPEED, 2.7 / Game.TILE);
    }
    // go back to original position
    rightMotor.rotate(70, false);
    searcher.retrieveRing();
    // rotate the right motor to behind a little to make sure we can put the rod behind the ring
    // rightMotor.rotate(RIGHT_MOTOR_RING_COR, false);

    // go to the position where ring can be retrieved

    // rotate a little to the left to make sure not influence the other ring
    rightMotor.rotate(-70, false);

    forward(FORWARD_SPEED, -6.5 / Game.TILE);
    // go back to original position
    rightMotor.rotate(40 + 30, false);

    // if (correct) {
    // forward(FORWARD_SPEED, -6.5 / Game.TILE);
    // } else {
    // forward(FORWARD_SPEED, -6 / Game.TILE);
    // }
    // if (reset)
    // searcher.resetRodMotor();
  }

  /**
   * This method rotates the robot by a certain angle
   * 
   * @param angle The angle (in degrees) to rotate our robot to
   */
  public void turn(int angle) {
    leftMotor.rotate(convertAngle(Game.WHEEL_RAD, Game.TRACK, angle), true);
    rightMotor.rotate(-convertAngle(Game.WHEEL_RAD, Game.TRACK, angle), false);
  }

  /**
   * This method moves our robot forward
   * 
   * @param speed An integer that denotes the speed to rotate our wheels at
   * @param distance A double that denotes the distance to travel
   */
  public void forward(int speed, double distance) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    leftMotor.rotate(convertDistance(Game.WHEEL_RAD, distance * Game.TILE), true);
    rightMotor.rotate(convertDistance(Game.WHEEL_RAD, distance * Game.TILE), false);
  }

  /**
   * This method stops both motors
   */
  public void stop() {
    leftMotor.stop(true);
    rightMotor.stop(false);
  }

  /**
   * This method converts a distance to the total number of wheel rotations needed to cover that
   * distance.
   * 
   * @param radius The radius of our wheels
   * @param distance The distance to travel
   * @return A number of wheel rotations
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This method converts an angle value to the number of wheel rotations needed to achieve rotation
   * 
   * @param radius The radius of our wheels
   * @param width The width
   * @param angle The angle to convert
   * @return A number of wheel rotations
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
