package ca.mcgill.ecse211.project;

/**
 * This class is used to check the color of a ring under a light sensor
 * 
 * @author Caspar Cedro
 * @author Percy Chen
 * @author Patrick Erath
 * @author Anssam Ghezala
 * @author Susan Matuszewski
 * @author Kamy Moussavi Kafi
 */
public class ColorCalibrator {
  private static volatile Color currentColor;
  private static int [] colour_frequency = {0, 0, 0, 0, 0};

  /**
   * This enumeration contains the possible colors of the ring under a light sensor
   */
  public static enum Color {
    Orange, Green, Blue, Yellow, Other
  }
  //this is v1

  private final static int lowerYellowRBound = 9, upperYellowRBound = 14, lowerYellowGBound = 4,
      upperYellowGBound = 10, lowerYellowBBound = 0, upperYellowBBound = 2, lowerBlueRBound = 0,
      upperBlueRBound = 1, lowerBlueGBound = 6, upperBlueGBound = 8, lowerBlueBBound = 3,
      upperBlueBBound = 8, lowerGreenRBound = 1, upperGreenRBound = 8, lowerGreenGBound = 5,
      upperGreenGBound = 9, lowerGreenBBound = 0, upperGreenBBound = 2, lowerOrangeRBound = 8,
      upperOrangeRBound = 20, lowerOrangeGBound = 0, upperOrangeGBound = 3, lowerOrangeBBound = 0, upperOrangeBBound = 1;

  /**
   * This method returns the color of the ring currently under the light sensor
   * 
   * @param r The red value to check for a ring
   * @param g The green value to check for a ring
   * @param b The blue value to check for a ring
   * @return A Color enumeration value
   */
  public static Color getColor(int r, int g, int b) {
    if (r>2*g && b<3 && r>lowerOrangeRBound ) {
      currentColor = Color.Orange;
    } else if (g>2*r && b<3) {
      currentColor = Color.Green;
      
      }else if ((r>= lowerYellowRBound && g >= lowerYellowGBound) || ((r>= 7 && r <= 9) && (g >=0 && g <= 2))) {
      currentColor = Color.Yellow;
    } else if ( (b >= lowerBlueBBound )) {
      currentColor = Color.Blue;
    } else {
      currentColor = Color.Other;
    }

    return currentColor;
  }
  /**
   * (r >= lowerGreenRBound && r <= 5)
        && (b >= lowerGreenBBound && b <= upperGreenBBound) && (g <= 12 && g >= 6)
        GREEN
   */

  /**
   * This method gets the last color of the ring under the light sensor
   * 
   * @return current color detected by the lightSensor
   */
  public static Color getColor() {
    if (currentColor != null)
      return currentColor;
    else
      return Color.Other;
  }
  
  /** 
   * This method keeps track of how many of each colour were detected
   * by increasing the count in the array
   * @param c The Color detected by the light sensor
   */
  public static void setFrequency(Color c) {
    switch (c) {
      case Blue:
        colour_frequency[1] ++;
        break;
      case Green:
        colour_frequency[2] ++;
        break;
      case Yellow:
        colour_frequency[3] ++;
        break;
      case Orange:
        colour_frequency[4] ++;
      default:
        break;
    }
  }
  
  /**
   * This method returns the most frequent colour detected from multiple samples
   * 
   * @return most frequent colour detected
   */
  public static Color getMostFrequenct() {
    Color c = Color.Other;
    int frequency = colour_frequency[0];
    for (int i = 0; i < colour_frequency.length; i++) {
      if (colour_frequency[i] >= frequency) {
        frequency = colour_frequency[i];
        c = getGetColor(i);
      }
    }
    if(frequency == 0) {
      c = Color.Other;
    }
    resetFrequency();
    return c;
  }
  
  /**
   * This method resets the colour_frequency array to 0
   * 
   */
  public static void resetFrequency() {
    for (int i = 0; i < colour_frequency.length; i ++) {
      colour_frequency[i] = 0;
    }
  }
  public static Color getGetColor(int i) {
    Color c = Color.Other;
    switch (i) {
      case 0:
        c = Color.Other;
        break;
      case 1: 
        c = Color.Blue;
        break;
      case 2:
        c = Color.Green;
        break;
      case 3:
        c = Color.Yellow;
        break;
      case 4:
        c = Color.Orange;
        break;
    }
    return c;
  }
}
