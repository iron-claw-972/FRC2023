package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Mech2d representation of the robot mechanism and grid.
 * 
 * <p> The construction can be chained.
 * 
 * <p> Maintaining the Mech2d should be inexpensive: values should only be transmitted if they are changed,
 * only a few values are changed (distance, elevator length, and fourbar angle),
 * and those value only need to change rarely.
 * The elevator length and fourbar angle might only be changed when their setpoints are set
 * rather than trying to track their incremental movement.
 * 
 * <p> See https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
 */
public class Mechanism {
    // the display (there is only one)
    static private Mechanism m_instance = null;

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

    /** the FourBar linkage that pivots. Set the angle of this linkage. */
    MechanismLigament2d m_fb2;

    /** the intake linkage that pivots off the FourBar */
    MechanismLigament2d m_intake;

    private Mechanism() {
        // define colors for the linkages
        Color8Bit colorSpace = new Color8Bit(0, 0, 0);
        Color8Bit colorNode = new Color8Bit(0, 255, 0);
        Color8Bit colorElevator = new Color8Bit(255, 128, 128);
        Color8Bit colorFourBar = new Color8Bit(255, 255, 0);
        Color8Bit colorIntake = new Color8Bit(128, 128, 255);
        Color8Bit colorShelf = new Color8Bit(255, 0, 0);
        Color8Bit colorDist =  new Color8Bit(128, 128, 128);

        // Build the grid

        // the entire grid is built relative to m_distGrid so the grid can be moved relative to the robot
        // distance of the grid from the root robot center (root)
        m_distGrid = new MechanismLigament2d("distGrid", 18, 0, 1.0, colorDist);
        m_root.append(m_distGrid);

        // front of grid is 5 inches high
        MechanismLigament2d front = new MechanismLigament2d("front", 5.0, 90.0, 2.0, colorNode);
        // connect it to the grid
        m_distGrid.append(front);

        // details of the shelves are not clear to me. Fake for now
        // move from front of grid to low shelf. Distance not clear.
        MechanismLigament2d spaceShelf = new MechanismLigament2d("spaceShelf", 12, 0, 0.25, colorSpace);
        m_distGrid.append(spaceShelf);
        // the shelf is 1 foot 11.5 inches (23.5 inches) high
        MechanismLigament2d riserLow = new MechanismLigament2d("lowRiser", 23.5, 90, 1.0, colorShelf);
        spaceShelf.append(riserLow);
        // unclear how deep the shelf is
        MechanismLigament2d shelfLow = new MechanismLigament2d("shelfLow", 24, -90, 1.0, colorShelf);
        riserLow.append(shelfLow);
        // the high riser reaches to 2 feet 11.5 inches (35.5 inches - 23.5 inches = 12 inches)
        MechanismLigament2d riserHigh = new MechanismLigament2d("riserHigh", 12, 90, 1.0, colorShelf);
        shelfLow.append(riserHigh);
        // unclear how deep the top shelf is
        MechanismLigament2d shelfHigh = new MechanismLigament2d("shelfHi", 12, -90, 1.0, colorShelf);
        riserHigh.append(shelfHigh);
        // shelf ends at alliance wall
        MechanismLigament2d wall = new MechanismLigament2d("wall", 36, 90, 4.0, colorShelf);
        shelfHigh.append(wall);

        // move from the grid to the origin of the middle node
        MechanismLigament2d spaceMiddle = new MechanismLigament2d("spaceMiddle", 22.75, 0.0, 0.25, colorSpace);
        m_distGrid.append(spaceMiddle);
        // make the middle node (2 feet 10 inches tall = 34 inches)
        MechanismLigament2d nodeMiddle = new MechanismLigament2d("nodeMiddle", 34.0, 90.0, 2.0, colorNode);
        // attach the middle node
        spaceMiddle.append(nodeMiddle);

        // move from front of grid to the origin of the high node
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
        MechanismLigament2d bumper1 = new MechanismLigament2d("bumper1", 13, 0);
        MechanismLigament2d bumper2 = new MechanismLigament2d("bumper2", 13, 180);
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
        spaceElevatorUp.append(m_elevator);

        // make the four bar (fake for now)
        // fb1 is portion that is horizontal and extends out from elevator carriage
        MechanismLigament2d fb1 = new MechanismLigament2d("fb1", 13.5, -55, 3.0, colorFourBar);
        m_elevator.append(fb1);
        // m_fb2 is moving part of the FourBar (angle is read by the absolute encoder)
        m_fb2 = new MechanismLigament2d("fb2", 9.5, 150.0, 3.0, colorFourBar);
        fb1.append(m_fb2);

        // make the intake. Its angle will be a function of the m_fb2 angle
        m_intake = new MechanismLigament2d("intake", 13.5, -150.0 * (32.0/48.0), 4.0, colorIntake);
        m_fb2.append(m_intake);

        // set the four bar and intake angles to be consistent
        setFourBarAngle(144.0);

        // put the Mechanism2D on the dashboard
        SmartDashboard.putData("Mech2d", m_mech2d);
    }

    /**
     * There is only one display.
     * @return
     */
    static public Mechanism getInstance() {
        // if the instance has not been constructed
        if (m_instance == null) {
            // then make it
            m_instance = new Mechanism();
        }
        
        return m_instance;
    }

    /**
     * Set the distance of the robot from the grid.
     * @param dist
     */
    public void setDistanceToGrid(double distance) {
        // convert the distance to inches
        double distInches = Units.metersToInches(Math.abs(distance));

        // clamp to sensible values. Do not get too close or too far away.
        // The center of the robot can be frame length / 2 + bumper thickness away.
        // frame length is 26.
        // Say bumper thickness is 3.5 inches (slight compression)
        distInches = MathUtil.clamp(distInches, 16.5, 48.0);
        
        // set the display distance
        m_distGrid.setLength(distInches);
    }

    /**
     * Set the slant length of the elevator.
     * @param height slant height in meters
     */
    public void setElevatorHeight(double height) {
        m_elevator.setLength(Units.metersToInches(height) + 4.0);
    }

    /**
     * Placeholder for setting FourBar state.
     * The absolute encoder before competition read -0.4 to 0.0 revolutions.
     * That is -0.4 * 360 = -144 degrees to 0 degrees.
     * When m_fb2 is is pointing a bit down (20 degrees?), the intake angle is level.
     * @param angle
     */
    public void setFourBarAngle(double angle) {
        // restrict angle to actual range
        double ang = MathUtil.clamp(angle, 0.0, 144.0);

        // an angle of 0 is actually down 20 degrees
        m_fb2.setAngle(ang - 20.0);

        // 32 tooth to 48 tooth
        m_intake.setAngle(-ang * (32.0 / 48.0) + 20.0);
    }
}