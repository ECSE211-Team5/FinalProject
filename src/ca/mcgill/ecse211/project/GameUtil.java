package ca.mcgill.ecse211.project;

import java.util.ArrayList;
import java.util.LinkedList;

public class GameUtil {
  class PathFinder{
    int[] ll;
    int[] ur;
    int width, length;
    int[] points;
    
    public PathFinder(int[] ll, int[] ur) {
      this.ll = ll;
      this.ur = ur;
      width = ur[0] - ll[0];
      length = ur[1] - ll[1];
      this.points = new int[(width+1)*(length+1)];
    }
    
    public boolean tryFindPath(int[] cur, int[] destination, ArrayList<Character> instruction) {
      if(!isSafe(destination)) return false;
      int i = coordinateToIndex(cur);
      int j = coordinateToIndex(destination);
      if(i < 0 || i > points.length-1 || j < 0 || j > points.length) return false;
      boolean[] visited = new boolean[points.length];
      int[] parents = new int[points.length];
      for(int n = 0; n < points.length; n++) {
        visited[n] = false;
        parents[n] = -1;
      }
      
      //bfs to find the path
      bfs(i, j, visited, parents);
      if(!visited[j]) return false;
      
      int node = j;
      //backtrace to find the way to go to destination
      while(node != -1) {
        int parent = parents[node];
        if (parent == -1) break;
        if(parent == node + 1) {
          instruction.add(0, 'L');
        }else if(parent == node - 1) {
          instruction.add(0, 'R');
        }else if(parent == node + (width+1)) {
          instruction.add(0, 'D');
        }else {
          instruction.add(0, 'U');
        }
        node = parent;
      }
      
      return true;
    }
    
    private void bfs(int i, int j, boolean[] visited, int[] parents) {
      LinkedList<Integer> q = new LinkedList<Integer>();
      q.addLast(i);
      
      while(q.size() > 0) {
        int ele = q.removeFirst();
        visited[ele] = true;
        if(ele == j) break;
        for(int p : getChildren(ele)) {
          if(!visited[p]) {
            q.addLast(p);
            parents[p] = ele;
          }
        }
      }
      
    }
    
    private ArrayList<Integer> getChildren(int i) {
      ArrayList<Integer> children = new ArrayList<Integer>();
      int[] coor = indexToCoor(i);
      int[][] connectedP = {{coor[0] - 1, coor[1]} ,{coor[0] + 1, coor[1]},
                            {coor[0], coor[1] + 1} ,{coor[0], coor[1] - 1}};
      for(int[] p : connectedP) {
        if(p[0] >= ll[0] && p[0] <= ur[0] && p[1] >= ll[1] && p[1] <= ur[1] && isSafe(p)) {
          children.add(coordinateToIndex(p));
        }
      }
      return children;
    }
    
    private int coordinateToIndex(int coor[]) {
      int i = coor[0] - ll[0] + (width+1) * (coor[1] - ll[1]);
      return i;
    }
    
    private int[] indexToCoor(int i) {
      int[] coor = {i%(width+1) + ll[0], i/(width+1) + ll[1]};
      return coor;
    }
    
    
    
  }
  
  public static boolean isSafe(int[] coor) {
    int x = coor[0];
    int y = coor[1];
    boolean inTunnel = x > GameParameters.TN_LL[0] && x < GameParameters.TN_LL[1] 
                      && y > GameParameters.TN_UR[0] && y< GameParameters.TN_UR[1];
    boolean isTree = x == GameParameters.TREE_US[0] && y == GameParameters.TREE_US[1];
    boolean outBound = x <= 0 || x >= GameParameters.Grid_UR[0] || y <= 0 || y >= GameParameters.Grid_UR[1];
    if(inTunnel || isTree || outBound) {
      return false;
    }
    return true;
  }
  
  public static double[] average(int[] p1, int[] p2) {
    double[] result = new double[2];
    result[0] = (double)(p1[0]+p2[0])/2;
    result[1] = (double)(p1[1]+p2[1])/2;
    return result;
  }
  
  public static int distanceFromStartingPoint(int x, int y) {
    return (int)(Math.pow(GameParameters.SC[0] - x, 2) + Math.pow(GameParameters.SC[1] - y, 2));
  }
}
