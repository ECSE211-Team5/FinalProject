package ca.mcgill.ecse211.project;

import java.util.ArrayList;
import java.util.LinkedList;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

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
  public static PathFinder startingFinder;
  public static PathFinder searchingFinder;
  /**
   * Class that find the shortest path from one point to another with constraints 
   * inside one area
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
     * Constructor of the path finder, 
     * @param ll: the lower left corner of the area
     * @param ur: the upper right corner of the area
     */
    public PathFinder(int[] ll, int[] ur) {
      this.ll = ll;
      this.ur = ur;
      width = ur[0] - ll[0];
      length = ur[1] - ll[1];
      this.points = new int[(width + 1) * (length + 1)];
    }

    /**
     * Try find a path from cur to destination, return false if no such path exists
     * @param cur: current position of the robot
     * @param destination: destination inside one area of the robot
     * @param instruction: instruction that guide the robot to the destination if such path exists
     * @return: true if such path exists, false otherwise
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
          instruction.add('L');
        } else if (parent == node - 1) {
          instruction.add('R');
        } else if (parent == node + (width + 1)) {
          instruction.add('D');
        } else {
          instruction.add('U');
        }
        node = parent;
      }

      return true;
    }

    /**
     * bfs the area
     * @param i: starting node
     * @param j: ending node
     * @param visited: an array keeps track of visited nodes
     * @param parents: an array keeps track of parent of each node
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
     * get a children of one node based on the constraint (cannot be a tree, cannot be in the tunnel,
     * cannot be a wall etc)
     * @param i: the index representation of the node
     * @return a list of child, if any
     */
    private ArrayList<Integer> getChildren(int i) {
      ArrayList<Integer> children = new ArrayList<Integer>();
      int[] coor = indexToCoor(i);
      int[][] connectedP = {{coor[0] - 1, coor[1]}, {coor[0] + 1, coor[1]}, {coor[0], coor[1] + 1},
          {coor[0], coor[1] - 1}};
      for (int[] p : connectedP) {
        if (p[0] >= ll[0] && p[0] <= ur[0] && p[1] >= ll[1] && p[1] <= ur[1] && isSafe(p)) {
          children.add(coordinateToIndex(p));
        }
      }
      return children;
    }

    /**
     * transfer a coordinate to index representation
     * @param coor
     * @return
     */
    private int coordinateToIndex(int coor[]) {
      int i = coor[0] - ll[0] + (width + 1) * (coor[1] - ll[1]);
      return i;
    }

    /**
     * transfer a index representation to its coordinate
     * @param i
     * @return
     */
    private int[] indexToCoor(int i) {
      int[] coor = {i % (width + 1) + ll[0], i / (width + 1) + ll[1]};
      return coor;
    }
  }

  /**
   * check if one coordinate is safe based on (it is not a wall, tree or inside a tunnel)
   * @param coor: coordinate array
   * @return: true if safe, false otherwise
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
   * find the average of the two coordinates
   * @param p1
   * @param p2
   * @return
   */
  public static double[] average(int[] p1, int[] p2) {
    double[] result = new double[2];
    result[0] = (double) (p1[0] + p2[0]) / 2;
    result[1] = (double) (p1[1] + p2[1]) / 2;
    return result;
  }

  /**
   * find the distance from the starting point to two points
   * @param x
   * @param y
   * @return
   */
  public static double distanceFromRobot(int x, int y) {
    try {
      double[] position = Odometer.getOdometer().getXYT();
      return (Math.pow(Math.round(position[0]) - x, 2) + Math.pow(Math.round(position[1]) - y, 2));
    } catch (OdometerExceptions e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return Integer.MAX_VALUE;
  }
  
  /**
   * find closest point from a set of points
   * @param points: a set of points
   * @return
   */
  public static int findClosestPointToRobot(int[][] points) {
    int minIndex = 0;
    double distance = Integer.MAX_VALUE;
    
    for(int i = 0; i < points.length; i++) {
      double thisDistance = distanceFromRobot(points[i][0], points[i][1]);
      if(thisDistance < distance) {
        minIndex = i;
        distance = thisDistance;
      }
    }
    return minIndex;
  }
}
