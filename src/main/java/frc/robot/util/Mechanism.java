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
 * <p> Maintaining the Mech2d should be inexpensive: values should only be transmitted if they are changed,
 * only a few values are changed (distance, elevator length, and fourbar angle),
 * and those value only need to change rarely.
 * The elevator length and fourbar angle might only be changed when their setpoints are set
 * rather than trying to track their incremental movement.
 * 
 * <p> See https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
 */
public class Mechanism {
    /* Mechanism2d canvas -- uses inches */
    Mechanism2d m_mech2d = new Mechanism2d(72, 96);

    // put the root at the bottom 24 inches from the left
    MechanismRoot2d m_root = m_mech2d.getRoot("ChargedUp",24, 2);

    // how far away the grid is from the root
    MechanismLigament2d m_distGrid;

    // set the length of this element for the length of the elevator
    MechanismLigament2d m_elevator;

    public Mechanism() {
        // Build the grid
        // distance of the grid from the root
        m_distGrid = new MechanismLigament2d("distGrid", 18, 0, 1.0, new Color8Bit(128, 128, 128));
        m_root.append(m_distGrid);

        // front of grid is 5 inches high
        MechanismLigament2d front = new MechanismLigament2d("front", 5.0, 90.0, 2.0, new Color8Bit(255, 255, 255));
        // connect it to the grid
        m_distGrid.append(front);

        // move from root to the origin of the middle node
        MechanismLigament2d spaceMiddle = new MechanismLigament2d("spaceMiddle", 22.75, 0.0, 0.25, new Color8Bit(0, 0, 0));
        m_distGrid.append(spaceMiddle);
        // make the middle node
        MechanismLigament2d nodeMiddle = new MechanismLigament2d("nodeMiddle", 34.0, 90.0, 2.0, new Color8Bit(255, 255, 255));
        // attach the middle node
        spaceMiddle.append(nodeMiddle);

        // move from root to the origin of the high node
        MechanismLigament2d spaceHigh = new MechanismLigament2d("spaceHigh", 39.75,  0.0, 0.25, new Color8Bit(0, 0, 0));
        m_distGrid.append(spaceHigh);
        // make the high node
        MechanismLigament2d nodeHigh = new MechanismLigament2d("nodeHigh", 46.0, 90.0, 2.0, new Color8Bit(255, 255, 255));
        // attach the high node
        spaceHigh.append(nodeHigh);


        // build the robot.
        // the root should be the center of the robot, but it is not right now
        // the deck of the robot should be off the ground.
        // The robot should be described in more detail.
        // make the elevator
        m_elevator = new MechanismLigament2d("elevator", 50, 55, 3.0, new Color8Bit(255, 128, 128));
        m_root.append(m_elevator);

        // make the four bar (fake for now)
        MechanismLigament2d fb1 = new MechanismLigament2d("fb1", 12, -55, 3, new Color8Bit(255, 255, 0));
        m_elevator.append(fb1);

        // put the Mechanism2D on the dashboard
        SmartDashboard.putData("Mech2d", m_mech2d);
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

    }
}
