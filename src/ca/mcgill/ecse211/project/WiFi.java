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
  private static final String SERVER_IP = "192.168.2.1";

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
      String tunnel = "TNG_LL_x";
      
      
      // GameParameters.SC = ((Long) data.get("RedTeam")).intValue();
      GameParameters.RedTeam = ((Long) data.get("RedTeam")).intValue();
      GameParameters.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
 
      
      
      GameParameters.RedCorner = ((Long) data.get("RedCorner")).intValue();
      GameParameters.GreenCorner = ((Long) data.get("GreenCorner")).intValue();

      GameParameters.US_LL[0] = ((Long) data.get("Red_LL_x")).intValue();
      GameParameters.US_LL[1] = ((Long) data.get("Red_LL_y")).intValue();
      GameParameters.US_UR[0] = ((Long) data.get("Red_UR_x")).intValue();
      GameParameters.US_UR[1] = ((Long) data.get("Red_UR_y")).intValue();

      GameParameters.Green_LL[0] = ((Long) data.get("Green_LL_x")).intValue();
      GameParameters.Green_LL[1] = ((Long) data.get("Green_LL_y")).intValue();
      GameParameters.Green_UR[0] = ((Long) data.get("Green_UR_x")).intValue();
      GameParameters.Green_UR[1] = ((Long) data.get("Green_UR_y")).intValue();
      
      //GameParameters.Grid_LL[0] = ((Long) data.get("Green_LL_x")).intValue();
      //GameParameters.Grid_LL[1] = ((Long) data.get("Green_LL_y")).intValue();
      //GameParameters.Grid_UR[0] = ((Long) data.get("Green_UR_x")).intValue();
      //GameParameters.Grid_UR[1] = ((Long) data.get("Green_UR_y")).intValue();

      GameParameters.Island_LL[0] = ((Long) data.get("Island_LL_x")).intValue();
      GameParameters.Island_LL[1] = ((Long) data.get("Island_LL_y")).intValue();
      GameParameters.Island_UR[0] = ((Long) data.get("Island_UR_x")).intValue();
      GameParameters.Island_UR[1] = ((Long) data.get("Island_UR_y")).intValue();

      GameParameters.TN_LL[0] = ((Long) data.get("TNR_LL_x")).intValue();
      GameParameters.TN_LL[1] = ((Long) data.get("TNR_LL_y")).intValue();
      GameParameters.TN_UR[0] = ((Long) data.get("TNR_UR_x")).intValue();
      GameParameters.TN_UR[1] = ((Long) data.get("TNR_UR_y")).intValue();

      GameParameters.TNO_LL[0] = ((Long) data.get("TNG_LL_x")).intValue();
      GameParameters.TNO_LL[1] = ((Long) data.get("TNG_LL_y")).intValue();
      GameParameters.TNO_UR[0] = ((Long) data.get("TNG_UR_x")).intValue();
      GameParameters.TNO_UR[1] = ((Long) data.get("TNG_UR_y")).intValue();

      GameParameters.TR[0] = ((Long) data.get("TR_x")).intValue();
      GameParameters.TR[1] = ((Long) data.get("TR_y")).intValue();

      GameParameters.TG[0] = ((Long) data.get("TG_x")).intValue();
      GameParameters.TG[1] = ((Long) data.get("TG_y")).intValue();

      if (GameParameters.Demo == GameParameters.DemoType.Beta) {
        GameParameters.Grid_UR[0] = 8;
        GameParameters.Grid_UR[1] = 8;

        if (GameParameters.Green_UR[0] - GameParameters.Green_LL[0] < 2
            || GameParameters.Green_UR[0] - GameParameters.Green_LL[0] > 8
            || GameParameters.Green_UR[1] - GameParameters.Green_LL[1] < 2
            || GameParameters.Green_UR[1] - GameParameters.Green_LL[1] > 8) {
          throw new Exception("Green zone coordinates out of bounds");
        }

        if (GameParameters.Island_UR[0] - GameParameters.Island_LL[0] < 2
            || GameParameters.Island_UR[0] - GameParameters.Island_LL[0] > 8
            || GameParameters.Island_UR[1] - GameParameters.Island_LL[1] < 2
            || GameParameters.Island_UR[1] - GameParameters.Island_LL[1] > 8) {
          throw new Exception("Green zone coordinates out of bounds");
        }

        if (GameParameters.TN_UR[0] - GameParameters.TN_LL[0] < 1
            || GameParameters.TN_UR[0] - GameParameters.TN_LL[0] > 2
            || GameParameters.TN_UR[1] - GameParameters.TN_LL[1] < 1
            || GameParameters.TN_UR[1] - GameParameters.TN_LL[1] > 2) {
          throw new Exception("Green tunnel coordinates out of bounds");
        }

        if (GameParameters.TG[0] < 0 || GameParameters.TG[0] > 7 || GameParameters.TG[1] < 0
            || GameParameters.TG[1] > 7) {
          throw new Exception("Green tree coordinates out of bounds");
        }
      } else {
        GameParameters.Grid_UR[0] = 15;
        GameParameters.Grid_UR[1] = 9;

        if (GameParameters.US_UR[0] - GameParameters.US_LL[0] < 2
            || GameParameters.US_UR[0] - GameParameters.US_LL[0] > 10
            || GameParameters.US_UR[1] - GameParameters.US_LL[1] < 2
            || GameParameters.US_UR[1] - GameParameters.US_LL[1] > 10) {
          throw new Exception("Red zone coordinates out of bounds");
        }

        if (GameParameters.Green_UR[0] - GameParameters.Green_LL[0] < 2
            || GameParameters.Green_UR[0] - GameParameters.Green_LL[0] > 10
            || GameParameters.Green_UR[1] - GameParameters.Green_LL[1] < 2
            || GameParameters.Green_UR[1] - GameParameters.Green_LL[1] > 10) {
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

        if (GameParameters.TN_UR[0] - GameParameters.TN_LL[0] < 1
            || GameParameters.TN_UR[0] - GameParameters.TN_LL[0] > 2
            || GameParameters.TN_UR[1] - GameParameters.TN_LL[1] < 1
            || GameParameters.TN_UR[1] - GameParameters.TN_LL[1] > 2) {
          throw new Exception("Green tunnel coordinates out of bounds");
        }

        if (GameParameters.TR[0] < 0 || GameParameters.TR[0] > 14 || GameParameters.TR[1] < 0
            || GameParameters.TR[1] > 8) {
          throw new Exception("Red tree coordinates out of bounds");
        }

        if (GameParameters.TG[0] < 0 || GameParameters.TG[0] > 14 || GameParameters.TG[1] < 0
            || GameParameters.TG[1] > 8) {
          throw new Exception("Green tree coordinates out of bounds");
        }
      }
      
      //set SC here
      //GameParameters.SC[0];
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }
}
