package ca.mcgill.ecse211.project;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * This class implements the ability to receive game parameters over WiFi
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public enum WiFi {
  INSTANCE;
  
  // ** Set these as appropriate for your team and current situation **
  private static final String SERVER_IP = "192.168.2.16";

  /**
   * This method sets up a connection to a locally hosted server and reads Game Parameter values
   * from it.
   */
  public static void readData() {
    WifiConnection connection =
        new WifiConnection(SERVER_IP, GameParameters.PlayerTeamNumber, true);

    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = connection.getData();
      String tunnelUs = "TNR_";
      String tunnelO = "TNG_";
      String ringUs = "TR_";
      String ringO = "TG_";
      String startingUs = "Red_";
      String startingO = "Green_";
      String cornerUs = "RedCorner";
      String cornerO = "GreenCorner";
     
      GameParameters.RedTeam = ((Long) data.get("RedTeam")).intValue();
      GameParameters.GreenTeam = ((Long) data.get("GreenTeam")).intValue();

      if(GameParameters.RedTeam == GameParameters.PlayerTeamNumber) {
      }else if(GameParameters.GreenTeam == GameParameters.PlayerTeamNumber) {
        System.out.println("GREEEN");
        String s = tunnelO;
        tunnelO = tunnelUs;
        tunnelUs = s;
        //change ring set
        s = ringO;
        ringO = ringUs;
        ringUs = s;
        //change starting corner
        s = startingO;
        startingO = startingUs;
        startingUs = s;
        //change corner
        s = cornerO;
        cornerO = cornerUs;
        cornerUs = s;
        
      }else {
        throw new Exception("Bad Team Number");
      }
      
      
      GameParameters.UsCorner = ((Long) data.get(cornerUs)).intValue();
      GameParameters.OCorner = ((Long) data.get(cornerO)).intValue();

      GameParameters.US_LL[0] = ((Long) data.get(startingUs+"LL_x")).intValue();
      GameParameters.US_LL[1] = ((Long) data.get(startingUs+"LL_y")).intValue();
      GameParameters.US_UR[0] = ((Long) data.get(startingUs+"UR_x")).intValue();
      GameParameters.US_UR[1] = ((Long) data.get(startingUs+"UR_y")).intValue();

      GameParameters.OPPO_LL[0] = ((Long) data.get(startingO+"LL_x")).intValue();
      GameParameters.OPPO_LL[1] = ((Long) data.get(startingO+"LL_y")).intValue();
      GameParameters.OPPO_UR[0] = ((Long) data.get(startingO+"UR_x")).intValue();
      GameParameters.OPPO_UR[1] = ((Long) data.get(startingO+"UR_y")).intValue();
      
      //GameParameters.Grid_LL[0] = ((Long) data.get("Green_LL_x")).intValue();
      //GameParameters.Grid_LL[1] = ((Long) data.get("Green_LL_y")).intValue();
      //GameParameters.Grid_UR[0] = ((Long) data.get("Green_UR_x")).intValue();
      //GameParameters.Grid_UR[1] = ((Long) data.get("Green_UR_y")).intValue();

      GameParameters.Island_LL[0] = ((Long) data.get("Island_LL_x")).intValue();
      GameParameters.Island_LL[1] = ((Long) data.get("Island_LL_y")).intValue();
      GameParameters.Island_UR[0] = ((Long) data.get("Island_UR_x")).intValue();
      GameParameters.Island_UR[1] = ((Long) data.get("Island_UR_y")).intValue();

      GameParameters.TN_LL[0] = ((Long) data.get(tunnelUs+"LL_x")).intValue();
      GameParameters.TN_LL[1] = ((Long) data.get(tunnelUs+"LL_y")).intValue();
      GameParameters.TN_UR[0] = ((Long) data.get(tunnelUs+"UR_x")).intValue();
      GameParameters.TN_UR[1] = ((Long) data.get(tunnelUs+"UR_y")).intValue();
      System.out.println(" GameParameters.TN_LL[0] " +  GameParameters.TN_LL[0]);
      System.out.println(" GameParameters.TN_LL[1] " +  GameParameters.TN_LL[1]);
      System.out.println(" GameParameters.TN_UR[0] " +  GameParameters.TN_UR[0]);
      System.out.println(" GameParameters.TN_UR[1] " +  GameParameters.TN_UR[1]);


      GameParameters.TNO_LL[0] = ((Long) data.get(tunnelO+"LL_x")).intValue();
      GameParameters.TNO_LL[1] = ((Long) data.get(tunnelO+"LL_y")).intValue();
      GameParameters.TNO_UR[0] = ((Long) data.get(tunnelO+"UR_x")).intValue();
      GameParameters.TNO_UR[1] = ((Long) data.get(tunnelO+"UR_y")).intValue();

      GameParameters.TREE_US[0] = ((Long) data.get(ringUs+"x")).intValue();
      GameParameters.TREE_US[1] = ((Long) data.get(ringUs+"y")).intValue();

      GameParameters.TTEE_O[0] = ((Long) data.get(ringO+"x")).intValue();
      GameParameters.TTEE_O[1] = ((Long) data.get(ringO+"y")).intValue();

      if (GameParameters.Demo == GameParameters.DemoType.Beta) {
        GameParameters.Grid_UR[0] = 8;
        GameParameters.Grid_UR[1] = 8;
        
        switch(GameParameters.UsCorner) {
          case 0:
            int[] sc0 = {1,1,0};
            GameParameters.SC = sc0;
            break;
          case 1:
            int[] sc1 = {7,1,270};
            GameParameters.SC = sc1;
            break;
          case 2:
            int[] sc2 = {7,7,180};
            GameParameters.SC = sc2;
            break;
          case 3:
            int[] sc3 = {1,7,90};
            GameParameters.SC = sc3;
            break;
        }
/**
        if (GameParameters.OPPO_UR[0] - GameParameters.OPPO_LL[0] < 2
            || GameParameters.OPPO_UR[0] - GameParameters.OPPO_LL[0] > 8
            || GameParameters.OPPO_UR[1] - GameParameters.OPPO_LL[1] < 2
            || GameParameters.OPPO_UR[1] - GameParameters.OPPO_LL[1] > 8) {
          throw new Exception("Green zone coordinates out of bounds");
        }

        if (GameParameters.Island_UR[0] - GameParameters.Island_LL[0] < 2
            || GameParameters.Island_UR[0] - GameParameters.Island_LL[0] > 8
            || GameParameters.Island_UR[1] - GameParameters.Island_LL[1] < 2
            || GameParameters.Island_UR[1] - GameParameters.Island_LL[1] > 8) {
          throw new Exception("Green zone coordinates out of bounds");
        }


        if (GameParameters.TTEE_O[0] < 0 || GameParameters.TTEE_O[0] > 7 || GameParameters.TTEE_O[1] < 0
            || GameParameters.TTEE_O[1] > 7) {
          throw new Exception("Green tree coordinates out of bounds");
        }
        **/
      } else {
        GameParameters.Grid_UR[0] = 15;
        GameParameters.Grid_UR[1] = 9;
 
        switch(GameParameters.UsCorner) {
          case 0:
            int[] sc0 = {1,1,0};
            GameParameters.SC = sc0;
            break;
          case 1:
            int[] sc1 = {14,1,270};
            GameParameters.SC = sc1;
            break;
          case 2:
            int[] sc2 = {14,8,180};
            GameParameters.SC = sc2;
            break;
          case 3:
            int[] sc3 = {1,8,90};
            GameParameters.SC = sc3;
            break;
        }
      /**  
        if (GameParameters.US_UR[0] - GameParameters.US_LL[0] < 2
            || GameParameters.US_UR[0] - GameParameters.US_LL[0] > 10
            || GameParameters.US_UR[1] - GameParameters.US_LL[1] < 2
            || GameParameters.US_UR[1] - GameParameters.US_LL[1] > 10) {
          throw new Exception("Red zone coordinates out of bounds");
        }

        if (GameParameters.OPPO_UR[0] - GameParameters.OPPO_LL[0] < 2
            || GameParameters.OPPO_UR[0] - GameParameters.OPPO_LL[0] > 10
            || GameParameters.OPPO_UR[1] - GameParameters.OPPO_LL[1] < 2
            || GameParameters.OPPO_UR[1] - GameParameters.OPPO_LL[1] > 10) {
          throw new Exception("Green zone coordinates out of bounds");
        }

        if (GameParameters.Island_UR[0] - GameParameters.Island_LL[0] < 2
            || GameParameters.Island_UR[0] - GameParameters.Island_LL[0] > 10
            || GameParameters.Island_UR[1] - GameParameters.Island_LL[1] < 2
            || GameParameters.Island_UR[1] - GameParameters.Island_LL[1] > 10) {
          throw new Exception("Island zone coordinates out of bounds");
        }

        if (GameParameters.TN_UR[0] - GameParameters.TN_LL[0] < 1
            || GameParameters.TN_UR[0] - GameParameters.TN_LL[0] > 2
            || GameParameters.TN_UR[1] - GameParameters.TN_LL[1] < 1
            || GameParameters.TN_UR[1] - GameParameters.TN_LL[1] > 2) {
          throw new Exception("Red tunnel coordinates out of bounds");
        }


        if (GameParameters.TREE_US[0] < 0 || GameParameters.TREE_US[0] > 14 || GameParameters.TREE_US[1] < 0
            || GameParameters.TREE_US[1] > 8) {
          throw new Exception("Red tree coordinates out of bounds");
        }

        if (GameParameters.TTEE_O[0] < 0 || GameParameters.TTEE_O[0] > 14 || GameParameters.TTEE_O[1] < 0
            || GameParameters.TTEE_O[1] > 8) {
          throw new Exception("Green tree coordinates out of bounds");
        }**/
      }
      
      //set data read to true
      synchronized(GameParameters.waitDataObject) {
        GameParameters.hasDataRead = true;
        GameParameters.waitDataObject.notify();
      }
      
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
