package ca.mcgill.ecse211.project;

/**
 * This singleton contains all the game parameters needed for the competition
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public enum GameParameters {
  INSTANCE;
  
  /**
   * This enumeration stores the possible types of areas that our robot is currently located in
   */
  public enum AreaType{
    Starting, Searching, Dangerous, StartingBoundary, SearchingBoundary
  }
  
  /**
   * This variable stores the current area that our robot is in during a competition
   */
  public static AreaType area = AreaType.Starting;
  
  /**
   * This variables holds the starting corner coordinates for our robot
   */
  public static int[] SC = {0, 0};
  
  /**
   * This variable stores the lower left coordinates of the entire grid
   */
  public static int[] Grid_LL = {0, 0};

  /**
   * This variable stores the upper right coordinates of the entire grid
   */
  public static int[] Grid_UR = {15, 9};

  /**
   * This variable holds the color of the target ring in the range [1,4]
   * 1 indicates a BLUE ring
   * 2 indicates a GREEN ring
   * 3 indicates a YELLOW ring
   * 4 indicates an ORANGE ring
   */
  public static int TR = -1;

  /**
   * This variable stores the number of the team our robot is on
   */
  public static int PlayerTeamNumber = -1;

  /**
   * This variable stores the team starting out from the red zone, possible values are [1,20]
   */
  public static int RedTeam = -1;
  
  /**
   * This variable stores the team starting out from the green zone, possible values are [1,20]
   */
  public static int GreenTeam = -1;
  
  /**
   * This variable stores the starting corner for the red team, possible values are [0,3]
   */
  public static int RedCorner = -1;
  
  /**
   * This variable stores the starting corner for the green team, possible values are [0,3]
   */
  public static int GreenCorner = -1;

  /**
   * This variable stores the lower left hand corner of the red zone
   * [0] = x coordinate, [1] = y coordinate
   * min Red_UR[0] - Red_LL[0] = 2
   * max Red_UR[0] - Red_LL[0] = 10 
   * min Red_UR[1] - Red_LL[1] = 2
   * max Red_UR[1] - Red_LL[1] = 10 
   */
  public static int[] Red_LL = {0, 5};

  /**
   * This variable stores the upper right hand corner of the red zone
   * [0] = x coordinate, [1] = y coordinate
   * min Red_UR[0] - Red_LL[0] = 2
   * max Red_UR[0] - Red_LL[0] = 10 
   * min Red_UR[1] - Red_LL[1] = 2
   * max Red_UR[1] - Red_LL[1] = 10 
   */
  public static int[] Red_UR = {4, 9};
  
  /**
   * This variable stores the lower left hand corner of the green zone
   * [0] = x coordinate, [1] = y coordinate
   * min Green_UR[0] - Green_LL[0] = 2
   * max Green_UR[0] - Green_LL[0] = 10
   * min Green_UR[1] - Green_LL[1] = 2
   * max Green_UR[1] - Green_LL[1] = 10 
   */
  public static int[] Green_LL = {10, 0};

  /**
   * This variable stores the upper right hand corner of the green zone
   * [0] = x coordinate, [1] = y coordinate
   * min Green_UR[0] - Green_LL[0] = 2
   * max Green_UR[0] - Green_LL[0] = 10
   * min Green_UR[1] - Green_LL[1] = 2
   * max Green_UR[1] - Green_LL[1] = 10
   */
  public static int[] Green_UR = {15, 4};

  /**
   * This variable stores the lower left hand corner of the red tunnel footprint
   * [0] = x coordinate, [1] = y coordinate
   * min BRR_UR[0] - BRR_LL[0] = 1
   * max BRR_UR[0] - BRR_LL[0] = 2 
   * min BRR_UR[1] - BRR_LL[1] = 1
   * max BRR_UR[1] - BRR_LL[1] = 2 
   */
  public static int[] BRR_LL = {4, 7};

  /**
   * This variable stores the upper right hand corner of the red tunnel footprint
   * [0] = x coordinate, [1] = y coordinate
   * min BRR_UR[0] - BRR_LL[0] = 1
   * max BRR_UR[0] - BRR_LL[0] = 2 
   * min BRR_UR[1] - BRR_LL[1] = 1
   * max BRR_UR[1] - BRR_LL[1] = 2 
   */
  public static int[] BRR_UR = {6, 8};

  /**
   * This variable stores the lower left hand corner of the green tunnel footprint
   * [0] = x coordinate, [1] = y coordinate
   * min BRG_UR[0] - BRG_LL[0] = 1
   * max BRG_UR[0] - BRG_LL[0] = 2
   * min BRG_UR[1] - BRG_LL[1] = 1
   * max BRG_UR[1] - BRG_LL[1] = 2 
   */
  public static int[] BRG_LL = {10, 3};

  /**
   * This variable stores the upper right hand corner of the green tunnel footprint
   * [0] = x coordinate, [1] = y coordinate
   * min BRG_UR[0] - BRG_LL[0] = 1
   * max BRG_UR[0] - BRG_LL[0] = 2
   * min BRG_UR[1] - BRG_LL[1] = 1
   * max BRG_UR[1] - BRG_LL[1] = 2 
   */
  public static int[] BRG_UR = {11, 5};

  /**
   * This variable stores the lower left hand corner of the red player ring set
   * [0] = x coordinate, [1] = y coordinate
   * min TR_UR[0] - TR_LL[0] = 1
   * max TR_UR[0] - TR_LL[0] = 1
   * min TR_UR[1] - TR_LL[1] = 1
   * max TR_UR[1] - TR_LL[1] = 1
   */
  public static int[] TR_LL = {7, 6};

  /**
   * This variable stores the upper right hand corner of the red player ring set
   * [0] = x coordinate, [1] = y coordinate
   * min TR_UR[0] - TR_LL[0] = 1
   * max TR_UR[0] - TR_LL[0] = 1
   * min TR_UR[1] - TR_LL[1] = 1
   * max TR_UR[1] - TR_LL[1] = 1
   */
  public static int[] TR_UR = {8, 7};

  /**
   * This variable stores the lower left hand corner of the green player ring set
   * [0] = x coordinate, [1] = y coordinate
   * min TG_UR[0] - TG_LL[0] = 1
   * max TG_UR[0] - TG_LL[0] = 1
   * min TG_UR[1] - TG_LL[1] = 1
   * max TG_UR[1] - TG_LL[1] = 1
   */
  public static int[] TG_LL = {13, 7};

  /**
   * This variable stores the upper right hand corner of the green player ring set
   * [0] = x coordinate, [1] = y coordinate
   * min TG_UR[0] - TG_LL[0] = 1
   * max TG_UR[0] - TG_LL[0] = 1
   * min TG_UR[1] - TG_LL[1] = 1
   * max TG_UR[1] - TG_LL[1] = 1
   */
  public static int[] TG_UR = {14, 8};
  
  /**
   * Giving a coordinate, find the type of area it belongs to
   * @param x x coordinate
   * @param y y coordinate
   * @return the type of area the point belongs to
   */
  public static AreaType getType(double x, double y) {
    return area;
  }
}
