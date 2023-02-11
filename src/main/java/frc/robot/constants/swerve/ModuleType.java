package frc.robot.constants.swerve;

public enum ModuleType {
    FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, NONE;
    
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
                return this.toString();
        }
    }

    public int getID() {
        switch (this) {
            case FRONT_LEFT:
                return 0;
            case FRONT_RIGHT:
                return 1;
            case BACK_LEFT:
                return 2;
            case BACK_RIGHT:
                return 3;
            default:
                return -1;
        }
    }
}