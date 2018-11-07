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
  private static final int TEAM_NUMBER = 1;

  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
  
  private WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
  
  /**
   * read data from wifi
   */
  public void readData() {
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
      Map data = conn.getData();
      
      //GameParameters.SC = ((Long) data.get("RedTeam")).intValue();
      GameParameters.RedTeam = ((Long) data.get("RedTeam")).intValue();
      GameParameters.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
      GameParameters.RedCorner = ((Long) data.get("RedCorner")).intValue();
      GameParameters.GreenCorner = ((Long) data.get("GreenCorner")).intValue();

      GameParameters.Red_LL[0] = ((Long) data.get("Red_LL_x")).intValue();
      GameParameters.Red_LL[1] = ((Long) data.get("Red_LL_y")).intValue();
      GameParameters.Red_UR[0] = ((Long) data.get("Red_UR_x")).intValue();
      GameParameters.Red_UR[1] = ((Long) data.get("Red_UR_y")).intValue();
      
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

      GameParameters.BRR_LL[0] = ((Long) data.get("TNR_LL_x")).intValue();
      GameParameters.BRR_LL[1] = ((Long) data.get("TNR_LL_y")).intValue();
      GameParameters.BRR_UR[0] = ((Long) data.get("TNR_UR_x")).intValue();
      GameParameters.BRR_UR[1] = ((Long) data.get("TNR_UR_y")).intValue();
      
      GameParameters.BRG_LL[0] = ((Long) data.get("TNG_LL_x")).intValue();
      GameParameters.BRG_LL[1] = ((Long) data.get("TNG_LL_y")).intValue();
      GameParameters.BRG_UR[0] = ((Long) data.get("TNG_UR_x")).intValue();
      GameParameters.BRG_UR[1] = ((Long) data.get("TNG_UR_y")).intValue();
      
      GameParameters.TR_LL[0] = ((Long) data.get("TR_x")).intValue();
      GameParameters.TR_LL[1] = ((Long) data.get("TR_y")).intValue();
      GameParameters.TR_UR[0] = ((Long) data.get("Island_UR_x")).intValue();
      GameParameters.TR_UR[1] = ((Long) data.get("Island_UR_y")).intValue();
      
      GameParameters.TG_LL[0] = ((Long) data.get("TG_x")).intValue();
      GameParameters.TG_LL[1] = ((Long) data.get("TG_y")).intValue();
      GameParameters.TG_UR[0] = ((Long) data.get("Island_UR_x")).intValue();
      GameParameters.TG_UR[1] = ((Long) data.get("Island_UR_y")).intValue();
      
      // Example 1: Print out all received data
      System.out.println("Map:\n" + data);

      // Example 2 : Print out specific values
      int redTeam = ((Long) data.get("RedTeam")).intValue();
      System.out.println("Red Team: " + redTeam);

      int TR_x = ((Long) data.get("TR_x")).intValue();
      System.out.println("X component of the red ring tree: " + TR_x);

      // Example 3: Compare value
      int tn_ll_x =  ((Long) data.get("TNR_LL_x")).intValue();
      if (tn_ll_x < 5) {
        System.out.println("Red Tunnel LL corner X < 5");
      }
      else {
        System.out.println("Red Tunnel LL corner X >= 5");
      }

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }
}
