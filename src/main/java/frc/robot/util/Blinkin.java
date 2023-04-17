package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.Constants;

public class Blinkin {
  
  private static Spark m_ledController;
  private static double currColor = Colors.ORANGE.m_id;
  private static boolean isDualColor = false;
  private static double[] dualColors;
  private static int dualColorIndex = 0;
  private static final Timer m_timer = new Timer();
  private static boolean timerRunning = false;

  /**
   * gets the static instance of the Spark Max controller used for the Blinkin control
   */
  private static Spark getController() {
    if (m_ledController == null) m_ledController = new Spark(Constants.kBlinkinPort);
    return m_ledController;
  }
  
  /**
   * sends a double input to the Blinkin colilationg to a pattern/color to use
   * the double input's pattern/color is based on the table in the documentation
   * https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   * 
   * @param tableInput numerical input for Pattern/Color to use
   */
  public static void setColor(double tableInput) {
    isDualColor = false;
    currColor = tableInput;
  }
  /**
   * sets pattern/color for Blinkin using Blinkin enum so you don't need to reference the table for numerical ids in the documentation
   * @param color Pattern/Color to use
   */
  public static void setColor(Colors color) {
    isDualColor = false;
    currColor = color.m_id;
  }

  /**
   * Sets colors for dual color mode. These colors will be cycled on the Blinkin every 500 milliseconds
   * @param color1 First color to be displayed
   * @param color2 Second color to be displayed
   */
  public static void setDualColor(Colors color1, Colors color2) {
    isDualColor = true;
    dualColors = new double[] {color1.m_id, color2.m_id};
    getController().set(dualColors[0]);
  }

  /**
   * Blinks a single color on the LEDs.
   * @param color
   */
  public static void blinkColor(Colors color) {
    setDualColor(color, Colors.BLACK);
  }

  /**
   * Blinks team colors, orange and black, on Blinkin. Black will turn LEDs off, so really only orange is shown.
   */
  public static void blinkTeamColors() {
    setDualColor(Colors.ORANGE, Colors.BLACK);
  }

  /**
   * Periodic method for cycling colors on Blinkin. Checks if timer has passed 0.5 seconds, and if it has
   * color will be changed.
   */
  private static void dualColorPeriodic() {
    m_timer.start();
    if (m_timer.advanceIfElapsed(0.5)) {
      if (dualColorIndex == 0) {
        getController().set(dualColors[1]);
        dualColorIndex = 1;
      } else if (dualColorIndex == 1) {
        getController().set(dualColors[0]);
        dualColorIndex = 0;
      }
    }
  }

  /**
   * Periodic method to update the LED colors on the blinkin
   */
  public static void colorPeriodic() {
    if (isDualColor) {
      dualColorPeriodic();
    } else {
      getController().set(currColor);
    }
  }

  /**
   * Blinkin enum contains the numerical ids so you don't need to reference the table for patterns/colors in the documentation
   * https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */
  public enum Colors {
    // Fixed Palette Pattern
    RAINBOW_RAINBOW_PALETTE(-0.99),
    RAINBOW_PARTY_PALETTE(-0.97),
    RAINBOW_OCEAN_PALETTE(-0.95),
    RAINBOW_LAVE_PALETTE(-0.93),
    RAINBOW_FOREST_PALETTE(-0.91),
    RAINBOW_GLITTER(-0.89),
    CONFETTI(-0.87),
    SHOT_RED(-0.85),
    SHOT_BLUE(-0.83),
    SHOT_WHITE(-0.81),
    SINELON_RAINBOW_PALETTE(-0.79),
    SINELON_PARTY_PALETTE(-0.77),
    SINELON_OCEAN_PALETTE(-0.75),
    SINELON_LAVE_PALETTE(-0.73),
    SINELON_FOREST_PALETTE(-0.71),
    BPM_RAINBOW(-0.69),
    BPM_PARTY(-0.67),
    BPM_OCEAN(-0.65),
    BPM_LAVE(-0.63),
    BPM_FOREST(-0.61),
    FIRE_MEDIUM(-0.59),
    FIRE_LARGE(-0.57),
    TWINKLES_RAINBOW_PALETTE(-0.55),
    TWINKLES_PARTY_PALETTE(-0.53),
    TWINKLES_OCEAN_PALETTE(-0.51),
    TWINKLES_LAVA_PALETTE(-0.49),
    TWINKLES_FOREST_PALETTE(-0.47),
    COLOR_WAVES_RAINBOW(-0.45),
    COLOR_WAVES_PARTY(-0.43),
    COLOR_WAVES_OCEAN(-0.41),
    COLOR_WAVES_LAVA(-0.39),
    COLOR_WAVES_FOREST(-0.37),
    LARSON_SCANNER_RED(-0.35),
    LARSON_SCANNER_GRAY(-0.33),
    LIGHT_CHASE_RED(-0.31),
    LIGHT_CHASE_BLUE(-0.29),
    LIGHT_CHASE_GRAY(-0.27),
    HEARTBEAT_RED(-0.25),
    HEARTBEAT_BLUE(-0.23),
    HEARTBEAT_WHITE(-0.21),
    HEARTBEAT_GRAY(-0.19),
    BREATH_RED(-0.17),
    BREATH_BLUE(-0.15),
    BREATH_GRAY(-0.13),
    STROBE_RED(-0.11),
    STROBE_BLUE(-0.9),
    STROBE_GOLD(-0.07),
    STROBE_WHITE(-0.05),
    // Color 1 Pattern
    END_TO_END_BLEND_BLACK_ONE(-0.03),
    LARSON_SCANNER_ONE(-0.01),
    LIGHT_CHASE_ONE(0.01),
    HEARTBEAT_SLOW_ONE(0.03),
    HEARTBEAT_MEDIUM_ONE(0.05),
    HEARTBEAT_FAST_ONE(0.07),
    BREATH_SLOW_ONE(0.09),
    BREATH_FAST_ONE(0.11),
    SHOT_ONE(0.13),
    STROBE_ONE(0.15),
    // Color 2 Pattern
    END_TO_END_BLEND_BLACK_TWO(0.17),
    LARSON_SCANNER_TWO(0.19),
    LIGHT_CHASE_TWO(0.21),
    HEARTBEAT_SLOW_TWO(0.23),
    HEARTBEAT_MEDIUM_TWO(0.25),
    HEARTBEAT_FAST_TWO(0.27),
    BREATH_SLOW_TWO(0.29),
    BREATH_FAST_TWO(0.31),
    SHOT_TWO(0.33),
    STROBE_TWO(0.35),
    // Color 1 and 2 Pattern
    SPARKLE_ONE_ON_TWO(0.37),
    SPARKLE_TWO_ON_ONE(0.39),
    COLOR_GRADIENT_ONE_AND_TWO(0.41),
    BPM_ONE_AND_TWO(0.43),
    END_TO_END_ONE_TO_TWO(0.45),
    END_TO_END(0.47),
    ONE_TWO_NO_BLENDING(0.49),
    TWINKLES_ONE_AND_TWO(0.51),
    COLOR_WAVES_ONE_AND_TWO(0.53),
    SINELON_ONE_AND_TWO(0.55),
    // Solid Color
    HOT_PINK(0.57),
    DARK_RED(0.59),
    RED(0.61),
    ORANGE(0.65),
    GOLD(0.67),
    YELLOW(0.69),
    LIME(0.73),
    GREEN(0.77),
    AQUA(0.81),
    SKY_BLUE(0.83),
    DARK_BLUE(0.85),
    BLUE(0.87),
    BLUE_VIOLET(0.89),
    VIOLET(0.91),
    WHITE(0.93),
    GRAY(0.95),
    DARK_GRAY(0.97),
    BLACK(0.99);

    public final double m_id;
    Colors(double id){
      m_id = id;
    }
  }
  
}
