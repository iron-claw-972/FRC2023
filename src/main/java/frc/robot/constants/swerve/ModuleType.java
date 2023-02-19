package frc.robot.constants.swerve;

/**
 * Represents the type for a module on the robot.
 */
public enum ModuleType {
  FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, NONE;
  
  /**
   * Gets the abbreviation for the swerve module type, ex: FL for FRONT_LEFT, BR for BACK_RIGHT.
   * @return the abbreviation for the swerve module type.
   */
  public String getAbbreviation() {
    switch (this) {
      case FRONT_LEFT:
        return "FL";
      case FRONT_RIGHT:
        return "FR";
      case BACK_LEFT:
        return "BL";
      case BACK_RIGHT:
        return "BR";
      default:
        return this.name();
    }
  }
  
  /**
   * Gets the ID for the swerve module type.
   * <p />
   * IDs:
   * 0 - FRONT_LEFT
   * 1 - FRONT_RIGHT
   * 2 - BACK_LEFT
   * 3 - BACK_RIGHT
   * @return the ID for the swerve module type.
   */
  public int getID() {
    if (this == NONE)
      return -1;
    return this.ordinal();
  }
}