package ca.mcgill.ecse211.project;

import java.util.ArrayList;
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
  private static final int FORWARD_SPEED = 140;
  private static final int ROTATE_SPEED = 80;
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
  
  private double calculateAngleTo(double x, double y) {
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
   * This method travel the robot to desired position by following the line (Always
   * rotate 90 degree), along with a correction
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
    double dX = x - odometer.getXYT()[0];
    double dY = y - odometer.getXYT()[1];
    // double theta = Math.atan(dX / dY);
    // if (dY < 0 && theta < Math.PI)
    // theta += Math.PI;

    // Euclidean distance calculation.
    // double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
    double theta = 0;
    if (dX > 0.1) {
      turnTo(90);
      theta = 90;
    } else if (dX < -0.1) {
      turnTo(-90);
      theta = -90;
    }
    moveWithCorrection(dX, theta);
    odometer.setX(x);
    
    if (dY > 0.1) {
      turnTo(0);
      theta = 0;
    } else if (dY < -0.1) {
      turnTo(180);
      theta = 180;
    }
    moveWithCorrection(dY, theta);
    odometer.setY(y);
  }

  /**
   * Move a certain distance with correction (using coordinate system)
   * @param distance: distance to cover
   * @param theta: theta to be corrected each time
   */
  public synchronized void moveWithCorrection(double distance, double theta) {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    //correct error of the distance
    int tiles = Math.abs((int)Math.round(distance));
    for (int i = 0; i < tiles; i++) {
      moveOneTileWithCorrection(theta);
    }
  }

  private void moveOneTileWithCorrection(double theta) {
    leftMotor.forward();
    rightMotor.forward();
    moveUntilLineDetection();
    odometer.setTheta(theta);
  }
  
  private void moveUntilLineDetection() {
    while (leftMotor.isMoving() || rightMotor.isMoving()) {
      double left = data.getL()[0];
      double right = data.getL()[1];
      if (left < -5) {
        leftMotor.stop(true);
      }

      if (right < -5) {
        rightMotor.stop(true);
      }
    }
    leftMotor.rotate(convertDistance(Game.WHEEL_RAD, Game.SEN_DIS), true);
    rightMotor.rotate(convertDistance(Game.WHEEL_RAD, Game.SEN_DIS), false);
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
      rightMotor.rotate(convertAngle(Game.WHEEL_RAD, Game.TRACK, 360 - dTheta) + 10, false);
    }
    // TURN LEFT
    else {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(convertAngle(Game.WHEEL_RAD, Game.TRACK, dTheta), true);
      rightMotor.rotate(-convertAngle(Game.WHEEL_RAD, Game.TRACK, dTheta) - 5, false);
    }
  }
  
  /**
   * found the tunnel based on the ll and ur coordinate, after the method, the robot will go the 
   * the entrance of the tunnel facing the tunnel it returns the distance it needs to go for [x] and [y]
   * in order to go through the tunnel
   */
  public void goThroughTunnel() {
    int distance = 0;
    int[] ll = GameParameters.TN_LL;
    int[] ur = GameParameters.TN_UR;
    int[] lr = {ll[0], ur[1]};
    int[] ul = {ur[0], ll[1]};
    ArrayList<int[]> notIn = new ArrayList<int[]>();
    int[][] corners = {ll, lr, ul, ur};
    ArrayList<int[]> points = new ArrayList<int[]>();
    for(int[] point : corners) {
      if(GameParameters.getType(point[0], point[1]) == GameParameters.AreaType.InStarting) {
        points.add(point);
      }else {
        notIn.add(point);
      }
    }
    if(points.get(0)[0] == points.get(1)[0]) {
      distance= Math.abs(notIn.get(0)[0] - points.get(0)[0]);

      travelToTunnel(points, 0);
      
    }else {
      distance= Math.abs(notIn.get(0)[1] - points.get(0)[1]); 
      travelToTunnel(points, 1);
    }
    
   
    double[] tunnelEnd = GameParameters.average(notIn.get(0), notIn.get(1));
    turnTo(Math.toDegrees(calculateAngleTo(tunnelEnd[0], tunnelEnd[1])));
    //goback To correct
    leftMotor.backward();
    rightMotor.backward();
    moveUntilLineDetection();
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    moveUntilLineDetection();
    forward(250, distance);
  }
  
  /**
   * 
   * @param n :0: x, 1: y
   */
  private void travelToTunnel(ArrayList<int[]> points, int n) {
    int[] closePoint = GameParameters.distanceFromStartingPoint(points.get(0)[0], points.get(0)[1]) > 
    GameParameters.distanceFromStartingPoint(points.get(1)[0], points.get(1)[1])? 
                                              points.get(1) : points.get(0);
    int[] plusOne = new int[2];
    int[] minusOne = new int[2];
      
     for(int i = 0; i < closePoint.length; i++) {
      if(i == n) {
        plusOne[n] = closePoint[n] + 1;
        minusOne[n] = closePoint[n] - 1;
      }else {
        plusOne[i] = closePoint[i];
        minusOne[i] = closePoint[i]; 
      }
    }
    boolean isMinus = GameParameters.distanceFromStartingPoint(plusOne[0], plusOne[1]) > 
    GameParameters.distanceFromStartingPoint(minusOne[0], minusOne[1]);
    int beforePoint[] = isMinus? minusOne : plusOne;
    double[] center = GameParameters.average(points.get(0), points.get(1));
    if(isMinus) {
      center[n]  = center[n] - 1;
    }else {
      center[n]  = center[n] + 1;
    }
    //travel to the point
    this.travelToWithCorrection(beforePoint[0], beforePoint[1], false);
    travelTo(center[0], center[1]);
  }
  
  /**
   * this method navigate the robot to the ring set, find the right position of the ring set 
   */
  public void goToRingSet() {
    
  }
  
  /**
   * this method approaches the ring set by paying attention to the reading of us sensor, stops
   * at the place when the robot can reach the ring
   */
  public void approachRingSet() {
    
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
  
  public void forward(int speed, int distance) {
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
