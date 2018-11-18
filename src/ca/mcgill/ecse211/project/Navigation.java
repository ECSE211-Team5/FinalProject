package ca.mcgill.ecse211.project;

import java.util.ArrayList;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.threads.RingSearcher;
import ca.mcgill.ecse211.threads.SensorData;
import lejos.hardware.Sound;
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
  private static final int FORWARD_SPEED = 140;
  private static final int ROTATE_SPEED = 250;
  private static final int ACCELERATION = 300;

  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private Odometer odometer;
  private SensorData data;

  /**
   * This navigation class constructor sets up our robot to begin navigating a particular map
   * 
   * @param leftMotor The EV3LargeRegulatedMotor instance for our left motor
   * @param rightMotor The EV3LargeRegulatedMotor instance for our right motor
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
      motor.setAcceleration(ACCELERATION);
    }
  }

  public double calculateAngleTo(double x, double y) {
    double dX = x - odometer.getXYT()[0];
    double dY = y - odometer.getXYT()[1];
    double theta = Math.atan(dX / dY);
    if (dY < 0 && theta < Math.PI)
      theta += Math.PI;
    return theta;
  }

  public void travelTo(double x, double y) {
    double dX = x - odometer.getXYT()[0];
    double dY = y - odometer.getXYT()[1];
    double theta = calculateAngleTo(x, y);

    // Euclidean distance calculation.
    double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));

    turnTo(Math.toDegrees(theta));

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(Game.WHEEL_RAD, distance * Game.TILE), true);
    rightMotor.rotate(convertDistance(Game.WHEEL_RAD, distance * Game.TILE), false);
  }

  /**
   * This method travel the robot to desired position by following the line (Always rotate 90
   * degree), along with a correction
   * 
   * When avoid=true, the nav thread will handle traveling. If you want to travel without avoidance,
   * this is also possible. In this case, the method in the Navigation class is used.
   * 
   * @param x The x coordinate to travel to (in cm)
   * @param y The y coordinate to travel to (in cm)
   * @param avoid: the robot will pay attention to the distance from ultrasonic sensor to avoid
   *        abstacle when navigating
   */
  public void travelToWithCorrection(int x, int y, boolean avoid) {
    int px = (int) Math.round(odometer.getXYT()[0]);
    int py = (int) Math.round(odometer.getXYT()[1]);
    int[] cur = {px, py};
    int[] destination = {x, y};
    ArrayList<Character> instruction = new ArrayList<Character>();
    
    //use path finder to find path based on different area the robot is at
    //OUT: instruction: contains a list of instruction for the robot to move to the destination
    if (GameParameters.getType(px, py) == GameParameters.AreaType.InStarting) {
      GameUtil.startingFinder.tryFindPath(cur, destination, instruction);
      
    } else {
      GameUtil.searchingFinder.tryFindPath(cur, destination, instruction);
    }
    
    //use the instruction modified by the pathFind to move to the destination
    char lastStep = ' ';
    int theta = 0;
    while (instruction.size() > 0) {
      char step = instruction.remove(instruction.size() - 1);
      //if the step is different from the last one, rotate to corresponding rotation
      if (step != lastStep) {
        theta = charToRotation(step);
        turnTo(theta);
      }
      if (step == 'L') {
        px--;
      } else if (step == 'R') {
        px++;
      } else if (step == 'U') {
        py++;
      } else {
        py--;
      }
      lastStep = step;
      moveWithCorrection(1, theta);
      odometer.setX(px);
      odometer.setY(py);
    }
  }

  private int charToRotation(char direction) {
    switch (direction) {
      case 'U':
        return 0;
      case 'D':
        return 180;
      case 'L':
        return 270;
      case 'R':
        return 90;
      default:
        return 0;
    }
  }

  /**
   * Move a certain distance with correction (using coordinate system)
   * 
   * @param distance: distance to cover
   * @param theta: theta to be corrected each time
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
  
  public synchronized void moveBackWithCorrection() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.backward();
    rightMotor.backward();
    moveUntilLineDetection(true);
  }

  public void moveOneTileWithCorrection(double theta) {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    moveUntilLineDetection(true);
    odometer.setTheta(theta);
  }

  private void moveUntilLineDetection(boolean checkForBlackLine) {
    double rotation = 0;
    int lastTachoCount = 0;
    char detacted = ' ';
    while (leftMotor.isMoving() || rightMotor.isMoving()) {
      double left = data.getL()[0];
      double right = data.getL()[1];
      if (left < -10) {
        leftMotor.stop(true);
        if(detacted == ' ') {
          detacted = 'l';
          rotation = odometer.getXYT()[2];
          lastTachoCount = leftMotor.getTachoCount();
        }
      }

      if (right < -13) {
        rightMotor.stop(true);
        if(detacted == ' ') {
          detacted = 'r';
          rotation = odometer.getXYT()[2];
          lastTachoCount = leftMotor.getTachoCount();
        }
      }
    }
    double afterRotation = odometer.getXYT()[2];
    boolean missedBlackLine = false;
    if(rotation < 10 || rotation > 350) {
      missedBlackLine = !(afterRotation > 340 || afterRotation < 20); 
    }else {
      missedBlackLine = Math.abs(afterRotation - rotation) > 30;
    }
    
    if(missedBlackLine && checkForBlackLine) {
      leftMotor.setSpeed(leftMotor.getMaxSpeed());
      rightMotor.setSpeed(rightMotor.getMaxSpeed());
      if(detacted == 'l') rightMotor.rotate(-(rightMotor.getTachoCount() - lastTachoCount));
      else if(detacted == 'r') leftMotor.rotate(-(leftMotor.getTachoCount() - lastTachoCount));
    }
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(Game.WHEEL_RAD, Game.SEN_DIS), true);
    rightMotor.rotate(convertDistance(Game.WHEEL_RAD, Game.SEN_DIS+0.5), false);
    if(missedBlackLine && checkForBlackLine) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.backward();
      rightMotor.backward();
      moveUntilLineDetection(false);
    }
  }

  /**
   * (*Improve* *Consider to discard*) This method is where the logic for the odometer will run. Use
   * the methods provided from the OdometerData class to implement the odometer.
   * 
   * @param angle The angle we want our robot to turn to (in degrees)
   * @param async whether return instantaneously
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
      leftMotor.rotate(-convertAngle(Game.WHEEL_RAD, Game.TRACK, 360 - dTheta), true);
      rightMotor.rotate(convertAngle(Game.WHEEL_RAD, Game.TRACK, 360 - dTheta), false);
    }
    // TURN LEFT
    else {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(convertAngle(Game.WHEEL_RAD, Game.TRACK, dTheta), true);
      rightMotor.rotate(-convertAngle(Game.WHEEL_RAD, Game.TRACK, dTheta), false);
    }
  }

  /**
   * found the tunnel based on the ll and ur coordinate, after the method, the robot will go the the
   * entrance of the tunnel facing the tunnel it returns the distance it needs to go for [x] and [y]
   * in order to go through the tunnel
   * 
   * @throws Exception
   */
  public void goThroughTunnel() throws Exception {
    int distance = 0;
    int[] ll, ur;
    //first use ll and ur coordinate to calculate lr and ul of the tunnel
    ll = GameParameters.TN_LL;
    ur = GameParameters.TN_UR;
    int[] lr = {ll[0], ur[1]};
    int[] ul = {ur[0], ll[1]};
    
    //clone the four points (to make sure we are not modifying the original one)
    int[][] corners = {ll.clone(), lr.clone(), ul.clone(), ur.clone()};
    ArrayList<int[]> notIn = new ArrayList<int[]>();
    ArrayList<int[]> points = new ArrayList<int[]>();
    double[] position = odometer.getXYT();
    
    //search for the points that are the same as the current area of the robot
    //these are the entrance of the tunnel, also find the other two points, those
    //are the exit of the tunnel
    for (int[] point : corners) {
      if (GameParameters.getType(point[0], point[1]) == GameParameters
          .getType((int) Math.round(position[0]), (int) Math.round(position[1]))) {
        points.add(point);
      } else {
        notIn.add(point);
      }
    }
    
    //find the direction and length of the tunnel
    //we know the entrance two points of the tunnel, so this means 
    //the two points must have either x or y coordinate identical.
    //that's the direction of the tunnel as well
    //after identify it's direction, we find whether it is positive 
    //or negative directed
    if (points.get(0)[0] == points.get(1)[0]) {
      distance = Math.abs(notIn.get(0)[0] - points.get(0)[0]) + 1;
      int multi = notIn.get(0)[0] - points.get(0)[0] < 0 ? 1 : -1;
      travelToTunnelEntrance(points, 0, multi);
      for (int i = 0; i < notIn.size(); i++) {
        //this step is to find the nearest two points that we can go two
        //after exit the tunnel
        notIn.get(i)[0] = notIn.get(i)[0] - multi * 1;
      }
    } else {
      distance = Math.abs(notIn.get(0)[1] - points.get(0)[1]) + 1;
      int multi = notIn.get(0)[1] - points.get(0)[1] < 0 ? 1 : -1;
      travelToTunnelEntrance(points, 1, multi);
      for (int i = 0; i < notIn.size(); i++) {
      //this step is to find the nearest two points that we can go two
      //after exit the tunnel
        notIn.get(i)[1] = notIn.get(i)[1] - multi * 1;
      }
    }

    
    double[] tunnelEnd = GameUtil.average(notIn.get(0), notIn.get(1));
    double angleThoughTunnel = Math.toDegrees(calculateAngleTo(tunnelEnd[0], tunnelEnd[1]));
    turnTo(angleThoughTunnel);
    
    // goback To correct
    moveBackWithCorrection();

    // turn left -6 to correct the effect of the weight
    turn(-5);
    forward(250, distance);
    odometer.setTheta(angleThoughTunnel);

    // rotate additional sensor distances to make sure the sensor will not on the balck line
    leftMotor.rotate(convertDistance(Game.WHEEL_RAD, 2*Game.SEN_DIS), true);
    rightMotor.rotate(convertDistance(Game.WHEEL_RAD, 2*Game.SEN_DIS), false);
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
   * This method navigate the robot to the entrance of the tunnel
   * @param n :0: x, 1: y
   */
  private void travelToTunnelEntrance(ArrayList<int[]> points, int n, int multiplier) {
    int[] safePoint = new int[2];
    for (int i = 0; i < points.size(); i++) {
      points.get(i)[n] = points.get(i)[n] + multiplier * 1;
    }
    //find the first safe point
    for(int[] p : points) {
      if(GameUtil.isSafe(p)) {
        safePoint = p;
        break;
      }
    }

    int[] beforePoint = safePoint;
    //find the nearst points to the entrance of the tunnel

    double[] center = GameUtil.average(points.get(0), points.get(1));

    // travel to the point
    this.travelToWithCorrection(beforePoint[0], beforePoint[1], false);
    travelTo(center[0], center[1]);
  }

  /**
   * this method navigate the robot to the ring set, find the right position of the ring set
   */
  public void goToRingSet() {

  }

  /**
   * this method approaches the ring set by paying attention to the reading of us sensor, stops at
   * the place when the robot can reach the ring
   */
  public void searchRingSet(RingSearcher searcher, boolean correct) {
    //Go backward to detect the line and correct the rotation
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    if(correct) {
      leftMotor.backward();
      rightMotor.backward();
      moveUntilLineDetection(true);
      //Forward for 3 cm (approach the ring set)
      forward(FORWARD_SPEED, 3/Game.TILE);
    }
    //rotate a little to the left to make sure that the sensor can detect the ring
    leftMotor.rotate(-30, false);
    //detect the ring color and beep based on the color
    searcher.search();
    //rotate back
    leftMotor.rotate(30, false);
    //prepare for retrieving the ring 
    searcher.prepareRetrieve();
    //go to the position where ring can be retrieved
    forward(FORWARD_SPEED, 5/Game.TILE);
    //rotate a little to the left to make sure not influence the other ring
    //leftMotor.rotate(-20, false);
    searcher.retrieveRing();
    //leftMotor.rotate(20, false);
    //go back to original position
    forward(FORWARD_SPEED, -8/Game.TILE);
    searcher.resetRodMotor();
  }

  /**
   * Rotate the robot by certain angle
   * 
   * @param angle The angle to rotate our robot to
   */
  public void turn(int angle) {
    leftMotor.rotate(convertAngle(Game.WHEEL_RAD, Game.TRACK, angle), true);
    rightMotor.rotate(-convertAngle(Game.WHEEL_RAD, Game.TRACK, angle), false);
  }

  /**
   * Move the robot forward
   * 
   * @param speed: speed to be taken
   * @param distance: distacne to travel
   */
  public void forward(int speed, double distance) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    leftMotor.rotate(convertDistance(Game.WHEEL_RAD, distance * Game.TILE), true);
    rightMotor.rotate(convertDistance(Game.WHEEL_RAD, distance * Game.TILE), false);
  }

  /**
   * Stop the motor
   */
  public void stop() {
    leftMotor.stop(true);
    rightMotor.stop(false);
  }

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius The radius of our wheels
   * @param distance The distance traveled
   * @return A converted distance
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This method allows the conversion of an angle value
   * 
   * @param radius The radius of our wheels
   * @param distance The distance traveled
   * @param angle The angle to convert
   * @return A converted angle
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
