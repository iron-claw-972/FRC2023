package frc.robot.constants.swerve;

/**
 * Represents the type for a module on the robot.
 */
public enum ModuleType {
  FRONT_LEFT(0, "FL"),
  FRONT_RIGHT(1, "FR"),
  BACK_LEFT(2, "BL"),
  BACK_RIGHT(3, "BR"),
  NONE(-1, "NONE");

  public final int id;
  public final String abbrev;

  ModuleType(int identifier, String abbreviation) {
    this.id = identifier;
    this.abbrev = abbreviation;
  }
  
  /**
   * Gets the abbreviation for the swerve module type, ex: FL for FRONT_LEFT, BR for BACK_RIGHT.
   * @deprecated Instead of ModuleType.getAbbreviation(), use ModuleType.abbrev
   * @return the abbreviation for the swerve module type.
   */
  @Deprecated
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
   * @deprecated Instead of ModuleType.getID(), used ModuleType.id
   * @return the ID for the swerve module type.
   */
  @Deprecated
  public int getID() {
    if (this == NONE)
      return -1;
    // This is a trick that relies on the order the enums are defined.
    return this.ordinal();
  }
}