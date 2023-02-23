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

    // how far away the grid is from the root
    MechanismLigament2d m_distGrid;

    // set the length of this element for the length of the elevator
    MechanismLigament2d m_elevator;

    // the FourBar
    MechanismLigament2d m_fb2;

    // the intake
    MechanismLigament2d m_intake;

    private Mechanism() {
        Color8Bit colorSpace = new Color8Bit(0, 0, 0);
        Color8Bit colorNode = new Color8Bit(0, 255, 0);
        Color8Bit colorElevator = new Color8Bit(255, 128, 128);
        Color8Bit colorFourBar = new Color8Bit(255, 255, 0);
        Color8Bit colorIntake = new Color8Bit(128, 128, 255);
        Color8Bit colorShelf = new Color8Bit(255, 0, 0);

        // Build the grid
        // distance of the grid from the root
        m_distGrid = new MechanismLigament2d("distGrid", 18, 0, 1.0, new Color8Bit(128, 128, 128));
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
        // the root should be the center of the robot, but it is not right now
        // the deck of the robot should be off the ground.
        // The robot should be described in more detail.
        // The Bumpers
        MechanismLigament2d bumper1 = new MechanismLigament2d("bumper1", 13, 0);
        MechanismLigament2d bumper2 = new MechanismLigament2d("bumper2", 13, 180);
        m_root.append(bumper1);
        m_root.append(bumper2);

        // make the elevator
        // elevator is 10 inches high and 12 inches back
        MechanismLigament2d spaceElevatorBack = new MechanismLigament2d("spaceEL", 12, 180, 0.2, colorSpace);
        m_root.append(spaceElevatorBack);
        MechanismLigament2d spaceElevatorUp = new MechanismLigament2d("elUp", 10, -90, 0.25, colorSpace);
        spaceElevatorBack.append(spaceElevatorUp);
        m_elevator = new MechanismLigament2d("elevator", 50, 55.0-90.0, 3.0, colorElevator);
        spaceElevatorUp.append(m_elevator);

        // make the four bar (fake for now)
        MechanismLigament2d fb1 = new MechanismLigament2d("fb1", 12, -55, 3.0, colorFourBar);
        m_elevator.append(fb1);
        // moving part of the FourBar
        m_fb2 = new MechanismLigament2d("fb2", 10, 150.0, 3.0, colorFourBar);
        fb1.append(m_fb2);

        // make the intake
        m_intake = new MechanismLigament2d("intake", 10, -150.0, 4.0, colorIntake);
        m_fb2.append(m_intake);

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
        
        m_distGrid.setLength(distInches);
    }

    /**
     * Set the slant length of the elevator.
     * @param height
     */
    public void setElevatorHeight(double height) {
        m_elevator.setLength(Units.metersToInches(height) + 4.0);
    }

    /**
     * Placeholder for setting FourBar state.
     * @param angle
     */
    public void setFourBarAngle(double angle) {
        // restrict angle to -170, 0
        double ang = MathUtil.clamp(angle, 0, 170);

        m_fb2.setAngle(ang);
        m_intake.setAngle(-ang);
    }
}
