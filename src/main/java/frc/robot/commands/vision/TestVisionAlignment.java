package frc.robot.commands.vision;

import java.util.ArrayList;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;
import frc.robot.util.Vision;

/**
 * Command to test alignment using vision
 * It only works if it can see an april tag at the setpoint
 */
public class TestVisionAlignment extends CommandBase{
  private Drivetrain m_drive;
  private Vision m_vision;
  private double m_setpoint;
  private double m_mostRecentAngle;

  /**
   * Constructs the TestVisionAlignment command
   * @param targetAngle The setpoint, in radians, that it should align to
   * @param drive The drivetrain
   * @param vision The vision
   */
  public TestVisionAlignment(double targetAngle, Drivetrain drive, Vision vision){
    addRequirements(drive);
    m_setpoint = targetAngle;
    m_drive = drive;
    m_vision = vision;
    m_mostRecentAngle = m_setpoint + Math.PI;
  }

  /**
   * Initializes the command
   */
  @Override
  public void initialize() {
    m_drive.getRotationController().reset();
  }

  /**
   * Runs the drivetrain's angle PID to get the the setpoint based on vision
   */
  @Override
  public void execute() {
    m_mostRecentAngle = getAngle();
    double speed = m_drive.getRotationController().calculate(m_mostRecentAngle, m_setpoint);
    m_drive.drive(0, 0, speed, false);
  }

  /**
   * Prints the vision pose and stops the robot
   * @param interrupted If the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    System.out.printf("\nExact angle: %.4f degrees\n", Units.radiansToDegrees(getAngle()));
    m_drive.stop();
  }

  /**
   * Returns if the command is finished
   * @return If the rotation PID is at the setpoint
   */
  @Override
  public boolean isFinished(){
    return m_drive.getRotationController().atSetpoint();
  }

  /**
   * Gets the angle from vision
   * @return The angle from the vision, the previous angle if it doensn't get a value
   */
  private double getAngle(){
    Pose2d pose = m_vision.getPose2d(m_drive.getPose());
    if (pose != null) {
      m_mostRecentAngle = pose.getRotation().getRadians();
    }
    return m_mostRecentAngle;
  }
}
