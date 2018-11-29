package ca.mcgill.ecse211.project;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;

/**
 * Game utility class with handy functionalities can be used in the game
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 *
 */
public class GameUtil {
  /**
   * This variable stores a PathFinder object instance that helps to create a path to navigate at
   * the start of a competition.
   */
  public static PathFinder startingFinder;

  /**
   * This variable stores a PathFinder object instance that helps to create a path to navigate
   * during the searching phase of a competition.
   */
  public static PathFinder searchingFinder;

  /**
   * This variable stores a character denoting a left turn is required.
   */
  public static char leftInstruction = 'L';

  /**
   * This variable stores a character denoting a right turn is required.
   */
  public static char rightInstruction = 'R';

  /**
   * This variable stores a character denoting that we need to navigate upwards.
   */
  public static char upInstruction = 'U';

  /**
   * This variable stores a character denoting that we need to navigate downwards.
   */
  public static char downInstruction = 'D';

  /**
   * This class is used to compare points nearest to our robot during sorting.
   * 
   * @author Caspar Cedro
   * @author Percy Chen
   * @author Patrick Erath
   * @author Anssam Ghezala
   * @author Susan Matuszewski
   * @author Kamy Moussavi Kafi
   *
   */
  public static class RobotComparator implements Comparator<int[]> {
    @Override
    public int compare(int[] a, int[] b) {
      double[] point = new double[2];
      try {
        point = Odometer.getOdometer().getXYT();
      } catch (OdometerExceptions e) {
        e.printStackTrace();
      }
      return (int) (GameUtil.distanceFrom(a[0], a[1], point)
          - GameUtil.distanceFrom(b[0], b[1], point));
    }
  }

  /**
   * This class is used to compare points nearest to a ring set during sorting.
   * 
   * @author Caspar Cedro
   * @author Percy Chen
   * @author Patrick Erath
   * @author Anssam Ghezala
   * @author Susan Matuszewski
   * @author Kamy Moussavi Kafi
   *
   */
  public static class RingSetComparator implements Comparator<int[]> {
    @Override
    public int compare(int[] a, int[] b) {
      double[] point = new double[2];
      for (int i = 0; i < point.length; i++) {
        point[i] = GameParameters.TREE_US[i];
      }
      return (int) (GameUtil.distanceFrom(a[0], a[1], point)
          - GameUtil.distanceFrom(b[0], b[1], point));
    }
  }

  /**
   * This class is used to compare points nearest to our robot's starting point during sorting.
   * 
   * @author Caspar Cedro
   * @author Percy Chen
   * @author Patrick Erath
   * @author Anssam Ghezala
   * @author Susan Matuszewski
   * @author Kamy Moussavi Kafi
   *
   */
  public static class StartingComparator implements Comparator<int[]> {
    @Override
    public int compare(int[] a, int[] b) {
      double[] point = new double[2];
      for (int i = 0; i < point.length; i++) {
        point[i] = GameParameters.SC[i];
      }
      return (int) (GameUtil.distanceFrom(a[0], a[1], point)
          - GameUtil.distanceFrom(b[0], b[1], point));
    }
  }

  /**
   * This class finds the shortest path from one point to another inside an area
   * 
   * @author Caspar Cedro
   * @author Percy Chen
   * @author Patrick Erath
   * @author Anssam Ghezala
   * @author Susan Matuszewski
   * @author Kamy Moussavi Kafi
   */
  public static class PathFinder {
    int[] ll;
    int[] ur;
    int width, length;
    int[] points;

    /**
     * This is the class constructor for the PathFinder class
     * 
     * @param ll The lower left corner of an area to find a path in
     * @param ur The upper right corner of an area to find a path in
     */
    public PathFinder(int[] ll, int[] ur) {
      this.ll = ll;
      this.ur = ur;
      width = ur[0] - ll[0];
      length = ur[1] - ll[1];
      this.points = new int[(width + 1) * (length + 1)];
    }

    /**
     * This method tries to find a path from our current location to our destination.
     * 
     * @param cur The current coordinates of our robot
     * @param destination The destination coordinates our robot is supposed to travel to
     * @param instruction The instructions that guide the robot to the destination if a path exists
     * @return A boolean that is true if a path exists otherwise false
     */
    public boolean tryFindPath(int[] cur, int[] destination, ArrayList<Character> instruction) {
      if (!isSafe(destination))
        return false;
      int i = coordinateToIndex(cur);
      int j = coordinateToIndex(destination);
      if (i < 0 || i > points.length - 1 || j < 0 || j > points.length)
        return false;
      boolean[] visited = new boolean[points.length];
      int[] parents = new int[points.length];
      for (int n = 0; n < points.length; n++) {
        visited[n] = false;
        parents[n] = -1;
      }

      // bfs to find the path
      bfs(i, j, visited, parents);
      if (!visited[j])
        return false;

      int node = j;
      // backtrace to find the way to go to destination
      while (node != -1) {
        int parent = parents[node];
        if (parent == -1)
          break;
        if (parent == node + 1) {
          instruction.add(leftInstruction);
        } else if (parent == node - 1) {
          instruction.add(rightInstruction);
        } else if (parent == node + (width + 1)) {
          instruction.add(downInstruction);
        } else {
          instruction.add(upInstruction);
        }
        node = parent;
      }

      return true;
    }

    /**
     * This method runs a breadth-first search on an area to find a potential path to a destination
     * 
     * @param i An integer that denotes the starting node
     * @param j An integer that denotes the ending node
     * @param visited An array keeps track of visited nodes
     * @param parents An array keeps track of the parent of each node
     */
    private void bfs(int i, int j, boolean[] visited, int[] parents) {
      LinkedList<Integer> q = new LinkedList<Integer>();
      q.addLast(i);

      while (q.size() > 0) {
        int ele = q.removeFirst();
        visited[ele] = true;
        if (ele == j)
          break;
        for (int p : getChildren(ele)) {
          if (!visited[p]) {
            q.addLast(p);
            parents[p] = ele;
          }
        }
      }

    }

    /**
     * This method gets the children of one node based on certain constraints (i.e. cannot be a
     * tree, cannot be in the tunnel, cannot be a wall etc)
     * 
     * @param i An integer that denotes the index representation of the node
     * @return A list of children, if any
     */
    private ArrayList<Integer> getChildren(int i) {
      ArrayList<Integer> children = new ArrayList<Integer>();
      int[] coor = indexToCoor(i);
      // get all points that are adjacent to p
      int[][] connectedP = {{coor[0] - 1, coor[1]}, {coor[0] + 1, coor[1]}, {coor[0], coor[1] + 1},
          {coor[0], coor[1] - 1}};
      for (int[] p : connectedP) {
        // check if the point is in the area and is safe
        if (p[0] >= ll[0] && p[0] <= ur[0] && p[1] >= ll[1] && p[1] <= ur[1] && isSafe(p)) {
          children.add(coordinateToIndex(p));
        }
      }
      return children;
    }

    /**
     * This method transforms a coordinate to square tile index representation
     * 
     * @param coor An array with x and y coordinates
     * @return The index of the square tile the coordinates represent
     */
    private int coordinateToIndex(int coor[]) {
      int i = coor[0] - ll[0] + (width + 1) * (coor[1] - ll[1]);
      return i;
    }

    /**
     * This method transforms a square tile index to its coordinate representation
     * 
     * @param i A square tile index
     * @return An array with x and y coordinates
     */
    private int[] indexToCoor(int i) {
      int[] coor = {i % (width + 1) + ll[0], i / (width + 1) + ll[1]};
      return coor;
    }
  }

  /**
   * This method checks if a coordinate pair is safe to travel to (i.e. it is not a wall, tree or
   * inside a tunnel)
   * 
   * @param coor An array with a pair of coordinates
   * @return A boolean that is true if safe, false otherwise
   */
  public static boolean isSafe(int[] coor) {
    int x = coor[0];
    int y = coor[1];
    boolean inTunnel = x >= GameParameters.TN_LL[0] && x <= GameParameters.TN_UR[0]
        && y >= GameParameters.TN_LL[1] && y <= GameParameters.TN_UR[1];
    boolean isTree = x == GameParameters.TREE_US[0] && y == GameParameters.TREE_US[1];
    boolean outBound =
        x <= 0 || x >= GameParameters.Grid_UR[0] || y <= 0 || y >= GameParameters.Grid_UR[1];
    if (inTunnel || isTree || outBound) {
      return false;
    }
    return true;
  }

  /**
   * This method finds the mid point between two coordinate pairs
   * 
   * @param p1 The first pair of coordinates
   * @param p2 The second pair of coordinates
   * @return An array with the mid point coordinates
   */
  public static double[] average(int[] p1, int[] p2) {
    double[] result = new double[2];
    result[0] = (double) (p1[0] + p2[0]) / 2;
    result[1] = (double) (p1[1] + p2[1]) / 2;
    return result;
  }

  /**
   * This method finds the distance from some starting coordinates to a target pair of coordinates
   * 
   * @param x The target x coordinate
   * @param y The target y coordinate
   * @param position The starting pair of coordinates
   * @return The distance from the starting position to the target coordinates
   */
  public static double distanceFrom(int x, int y, double[] position) {
    return (Math.pow(Math.round(position[0]) - x, 2) + Math.pow(Math.round(position[1]) - y, 2));
  }

  /**
   * This method finds the closest point from a set of points
   * 
   * @param points: a set of points
   * @return
   */
  public static int findClosestPointToRobot(int[][] points) {
    int minIndex = 0;
    double distance = Integer.MAX_VALUE;

    for (int i = 0; i < points.length; i++) {
      try {
        double[] point = Odometer.getOdometer().getXYT();
        double thisDistance = distanceFrom(points[i][0], points[i][1], point);
        if (thisDistance < distance) {
          minIndex = i;
          distance = thisDistance;
        }
      } catch (OdometerExceptions e) {
        e.printStackTrace();
      }
    }
    return minIndex;
  }

  /**
   * Check if a given point is boundary
   * 
   * @param coor: the point to input
   * @return: whether the given point is on the boundary of an island
   */
  public static boolean isIslandBoundary(int[] coor) {
    int x = coor[0];
    int y = coor[1];
    boolean onLY = x == GameParameters.Island_LL[0]
        && (y >= GameParameters.Island_LL[1] && y <= GameParameters.Island_UR[1]);
    boolean onRY = x == GameParameters.Island_UR[0]
        && (y >= GameParameters.Island_LL[1] && y <= GameParameters.Island_UR[1]);
    boolean onLX = y == GameParameters.Island_LL[1]
        && (x >= GameParameters.Island_LL[0] && x <= GameParameters.Island_UR[0]);
    boolean onUX = y == GameParameters.Island_UR[1]
        && (x >= GameParameters.Island_LL[0] && x <= GameParameters.Island_UR[0]);

    return onLY || onRY || onLX || onUX;
  }

  public static void playMusic() {
    Sound.playSample(new File("Pen.wav"), 100);
  }
}
