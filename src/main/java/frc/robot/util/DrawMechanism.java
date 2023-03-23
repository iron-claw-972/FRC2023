package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.RollerIntake.IntakeMode;

/**
 * Mech2d representation of the robot mechanism and grid.
 * 
 * <p> To use this class, get the instance with <code>DrawMechanism.getInstance()</code>.
 * 
 * <p> Then use methods such as <code>instance.setElevatorExtension()</code>.
 * 
 * <p> Maintaining the Mech2d should be inexpensive: values should only be transmitted if they are changed,
 * only a few values are changed (distance, elevator length, and wrist angle),
 * and those value only need to change rarely.
 * The elevator length and wrist angle might only be changed when their setpoints are set
 * rather than trying to track their incremental movement.
 * 
 * <p> See https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
 */
public class DrawMechanism {
    // the display (there is only one)
    static private DrawMechanism m_instance = null;

    /* Mechanism2d canvas -- uses inches */
    Mechanism2d m_mech2d = new Mechanism2d(72, 96);

    // the root is nominally the center of the robot
    //   put it at 18 inches
    //   make ground level be 2 inches from the bottom.
    MechanismRoot2d m_root = m_mech2d.getRoot("ChargedUp", 18, 2);

    /** how far away the grid is from the root (robot center) (inches) */
    MechanismLigament2d m_distGrid;

    /** this element is the length of the elevator (inches) */
    MechanismLigament2d m_elevator;

    /** the wrist linkage that pivots. Set the angle of this linkage. */
    MechanismLigament2d m_wrist;

    /** the intake linkage that pivots off the wrist */
    MechanismLigament2d m_intake;

    // a blueish color for disabled intake
    Color8Bit colorIntakeDisabled = new Color8Bit(128, 128, 255);
    // yellow color for cone intake/outtake - lighter is outtake
    Color8Bit colorIntakeConeIn = new Color8Bit(153, 163, 7);
    Color8Bit colorIntakeConeOut = new Color8Bit(207, 214, 90);
    // purple color for cube intake/outtake - lighter is outtake
    Color8Bit colorIntakeCubeIn = new Color8Bit(69, 1, 115);
    Color8Bit colorIntakeCubeOut = new Color8Bit(184, 93, 245);

    private DrawMechanism() {
        // define colors for the linkages
        Color8Bit colorSpace = new Color8Bit(0, 0, 0);
        Color8Bit colorNode = new Color8Bit(0, 255, 0);
        Color8Bit colorElevator = new Color8Bit(255, 128, 128);
        Color8Bit colorWrist = new Color8Bit(255, 255, 0);
        Color8Bit colorShelf = new Color8Bit(255, 0, 0);
        Color8Bit colorDist =  new Color8Bit(128, 128, 128);
        Color8Bit colorBumper = new Color8Bit(255, 0, 0);

        // Build the grid

        // the entire grid is built relative to m_distGrid so the grid can be moved relative to the robot
        // distance of the grid from the root robot center (root)
        m_distGrid = new MechanismLigament2d("distGrid", 13.0 + 3.5, 0, 1.0, colorDist);
        m_root.append(m_distGrid);

        // front of grid is 5 inches high
        MechanismLigament2d front = new MechanismLigament2d("front", 5.0, 90.0, 2.0, colorNode);
        // connect it to the grid
        m_distGrid.append(front);

        // details of the cube nodes.
        /*
         * Game Manual page 26:
         * Each CUBE NODE is a polycarbonate shelf that is 1 ft. 6¼ in. (~46 cm) wide and 1 ft. 5 in. (~43 cm) deep. 
         * CUBE NODES are surrounded by 3 in. (~8 cm) tall vertical walls, with the exception of the rear wall of the 
         * top ROW CUBE NODE which is angled. The distance from the FIELD carpet to the top of a middle ROW 
         * CUBE NODE wall is 1 ft. 11.5 in. (~60 cm). The distance from the FIELD carpet to the top of a top ROW 
         * CUBE NODE wall is 2 ft. 11.5 in. (~90 cm). The front of a middle ROW CUBE NODE is 1 ft. 2.25 in. (~36 cm) 
         * from the front face of the GRID. The front of a top ROW CUBE NODE is 2 ft. 7.625 in. (~80 cm) from the 
         * front face of the GRID.
         */
        // move from front of grid to middle row cube node is 1 foot 2.25 inches.
        MechanismLigament2d spaceShelf = new MechanismLigament2d("spaceShelf", 14.25, 0, 0.25, colorSpace);
        m_distGrid.append(spaceShelf);
        // the shelf is 1 foot 11.5 inches (23.5 inches) high
        MechanismLigament2d riserLow = new MechanismLigament2d("lowRiser", 23.5, 90, 1.0, colorShelf);
        spaceShelf.append(riserLow);
        // add the lip
        MechanismLigament2d lipMiddle = new MechanismLigament2d("lipMiddle", 3, 0, 0.25, colorShelf);
        riserLow.append(lipMiddle);
        // the shelf is 1 foot 5 inches deep
        //   does that check? NO!
        //   front of top row cube node is 2 feet 7 5/8 inches = 31.625 != 14.25 + 17 = 31.25 inches
        MechanismLigament2d shelfLow = new MechanismLigament2d("shelfLow", 17.0, -90, 1.0, colorShelf);
        riserLow.append(shelfLow);
        // the high riser reaches to 2 feet 11.5 inches (35.5 inches - 23.5 inches = 12 inches)
        MechanismLigament2d riserHigh = new MechanismLigament2d("riserHigh", 12, 90, 1.0, colorShelf);
        shelfLow.append(riserHigh);
        // add the lip
        MechanismLigament2d lipHigh = new MechanismLigament2d("lipHigh", 3, 0, 0.25, colorShelf);
        riserHigh.append(lipHigh);
        // a shelf is 1 foot 5 inches deep -- but back wall is angled?
        MechanismLigament2d shelfHigh = new MechanismLigament2d("shelfHi", 17.0, -90, 1.0, colorShelf);
        riserHigh.append(shelfHigh);
        // shelf ends at alliance wall
        // does this distance check out? NO! Short 8 inches (slanted back?)
        //    depth = 14.25 + 17.0 + 17.0 == 48.25 != depth of grid == 4 feet 8.25 inches == 56.25 inches
        MechanismLigament2d slantBack = new MechanismLigament2d("slantBack", 5*1.41, 45, 1.0, colorShelf);
        shelfHigh.append(slantBack);
        // 
        MechanismLigament2d wall = new MechanismLigament2d("wall", 36, 45, 4.0, colorShelf);
        slantBack.append(wall);

        // move from the grid to the origin of the middle cone node is 1 foot 10.75 inches
        MechanismLigament2d spaceMiddle = new MechanismLigament2d("spaceMiddle", 22.75, 0.0, 0.25, colorSpace);
        m_distGrid.append(spaceMiddle);
        // make the middle cone node (2 feet 10 inches tall = 34 inches)
        MechanismLigament2d nodeMiddle = new MechanismLigament2d("nodeMiddle", 34.0, 90.0, 2.0, colorNode);
        // attach the middle node
        spaceMiddle.append(nodeMiddle);

        // move from front of grid to the origin of the high cone node is 3 feet 3.75 inches
        MechanismLigament2d spaceHigh = new MechanismLigament2d("spaceHigh", 39.75,  0.0, 0.25, colorSpace);
        m_distGrid.append(spaceHigh);
        // make the high node (3 feet 10 inches tall = 46 inches)
        MechanismLigament2d nodeHigh = new MechanismLigament2d("nodeHigh", 46.0, 90.0, 2.0, colorNode);
        // attach the high node
        spaceHigh.append(nodeHigh);


        // build the robot.
        // the root should be the center of the robot
        // the deck of the robot should be off the ground.
        // The robot should be described in more detail.

        // The Bumpers
        // the robot is 26 inches by 26 inches.
        // Ignores bumper thickness for now. Should set width and color.
        MechanismLigament2d bumper1 = new MechanismLigament2d("bumper1", 13, 0, 4, colorBumper);
        MechanismLigament2d bumper2 = new MechanismLigament2d("bumper2", 13, 180, 4, colorBumper);
        m_root.append(bumper1);
        m_root.append(bumper2);

        // make the elevator
        // elevator is 10 inches high and 12 inches back
        // go back 12 inches
        MechanismLigament2d spaceElevatorBack = new MechanismLigament2d("spaceEL", 12, 180, 0.2, colorSpace);
        m_root.append(spaceElevatorBack);
        // go up 10 inches
        MechanismLigament2d spaceElevatorUp = new MechanismLigament2d("elUp", 10, -90, 0.25, colorSpace);
        spaceElevatorBack.append(spaceElevatorUp);
        // the elevator is slanted at 55 degrees.
        m_elevator = new MechanismLigament2d("elevator", 50, 55.0-90.0, 3.0, colorElevator);
        
        // elevator bottom stage is permanently there so appending it as a non-moving ligament. m_elevator will be the part that moves.
        MechanismLigament2d elevatorBar = new MechanismLigament2d("elevatorBar", 50, 55.0-90.0, 3.0, colorElevator);
        spaceElevatorUp.append(m_elevator);
        spaceElevatorUp.append(elevatorBar);

        // make the wrist
        // wristHandle is portion that is horizontal and extends out from elevator carriage
        MechanismLigament2d wristHandle = new MechanismLigament2d("wristHandle", 17, -55, 3.0, colorWrist);
        m_elevator.append(wristHandle);
        // m_wrist is moving part of the wrist (angle is read by the absolute encoder)
        m_wrist = new MechanismLigament2d("wrist", 3.5, 150.0, 3.0, colorWrist);
        wristHandle.append(m_wrist);

        // make the intake
        m_intake = new MechanismLigament2d("intake", 13, 0, 4.0, colorIntakeDisabled);
        m_wrist.append(m_intake);

        // set the wrist and intake angles to be consistent
        setWristAngle(WristConstants.kStowPos);

        // put the Mechanism2D on the dashboard
        SmartDashboard.putData("Mech2d", m_mech2d);

        // We have the Mech2d, so put some commands on the SmartDashboard
        addCommands(this);
    }

    /**
     * Add some commands to the SmartDashboard to test drawing the mechanism.
     * @param mechanism
     */
    private void addCommands(DrawMechanism mechanism) {
        // some commands to move the Mech2d diagram
        SmartDashboard.putData("wrist in", new InstantCommand(() -> mechanism.setWristAngle(WristConstants.kStowPos)));
        SmartDashboard.putData("wrist out", new InstantCommand(() -> mechanism.setWristAngle(WristConstants.kIntakeCubePos)));
        SmartDashboard.putData("elevator down", new InstantCommand(() -> mechanism.setElevatorExtension(0.1)));
        SmartDashboard.putData("elevator up", new InstantCommand(() -> mechanism.setElevatorExtension(1.3)));
        SmartDashboard.putData("drivetrain in", new InstantCommand(() -> mechanism.setDistanceToGrid(Units.inchesToMeters(16.5))));
        SmartDashboard.putData("drivetrain away", new InstantCommand(() -> mechanism.setDistanceToGrid(2)));
        SmartDashboard.putData("outtake cone", new InstantCommand(() -> mechanism.setIntakeStatus(IntakeMode.OUTTAKE_CONE)));
        SmartDashboard.putData("intake disable", new InstantCommand(() -> mechanism.setIntakeStatus(IntakeMode.DISABLED)));
    }

    /**
     * There is only one display.
     * @return
     */
    static public DrawMechanism getInstance() {
        // if the instance has not been constructed
        if (m_instance == null) {
            // then make it
            m_instance = new DrawMechanism();
        }
        
        return m_instance;
    }

    /**
     * Set the distance of the robot from the grid.
     * @param distance Distance from robot center to front of grid (meters)
     */
    public void setDistanceToGrid(double distance) {
        // convert the distance to inches
        double distInches = Units.metersToInches(Math.abs(distance));

        // clamp to sensible values. Do not get too close or too far away.
        // The center of the robot can be frame length / 2 + bumper thickness away.
        // frame length is 26.
        // Say bumper thickness is 3.5 inches (slight compression)
        // so center of robot to outside of front bumper is
        double distanceMin = 26.0 / 2 + 3.5;
        distInches = MathUtil.clamp(distInches, distanceMin, 48.0);
        
        // set the display distance
        m_distGrid.setLength(distInches);
    }

    /**
     * Set the slant length of the elevator.
     * @param length slant length in meters
     */
    public void setElevatorExtension(double length) {
        m_elevator.setLength(Units.metersToInches(length) + 4.0);
    }

    /**
     * Set the wrist angle
     * @param angle the angle in rotations, zero being forward
     */
    public void setWristAngle(double angle) {
        // restrict angle to actual range of the wrist
        double ang = MathUtil.clamp(angle, WristConstants.kMinPos, WristConstants.kMaxPos);

        // mech2d uses degrees
        m_wrist.setAngle(Units.rotationsToDegrees(ang));
    }

    public void setIntakeStatus(IntakeMode mode) {
        switch (mode) {
            case DISABLED:
                m_intake.setColor(colorIntakeDisabled);
                break;
            case INTAKE_CONE:
                m_intake.setColor(colorIntakeConeIn);
                break;
            case INTAKE_CUBE:
                m_intake.setColor(colorIntakeCubeIn);
                break;
            case OUTTAKE_CONE:
                m_intake.setColor(colorIntakeConeOut);
                break;
            case OUTTAKE_CUBE:
                m_intake.setColor(colorIntakeCubeOut);
                break;
            default:
                break;
        }
    }
}
