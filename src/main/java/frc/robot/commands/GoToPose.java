package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Tests the odometry of the robot by driving a certain distance and calculating the error.
 */
public class GoToPose extends CommandBase {

  private Drivetrain m_drive; 
  private Supplier<Pose2d> m_poseSupplier;
  private double m_startTime;
  private Pose2d m_finalPose;
  private Pose2d m_error;
  private boolean m_relativeToRobot;
  private boolean m_doNotEnd = false;
  
  /**
   * Returns a command that goes to a position based on shuffleboard inputs
   * DO NOT USE IF SHUFFLEBOARD IS DISABLED 
   * @param drive drivetrain to be used for the command
   * @param relativeToRobot should pose be converted to a transformation
   * @param doNotEnd should the robot not end for tunning purposes
   */
  public GoToPose(Drivetrain drive, boolean relativeToRobot, boolean doNotEnd){
    this(drive,
      () -> new Pose2d(
        drive.getRequestedXPos().getDouble(0), 
        drive.getRequestedYPos().getDouble(0), 
        new Rotation2d(drive.getRequestedHeadingEntry().getDouble(0))
      ),
      relativeToRobot
    );
    m_doNotEnd = doNotEnd;
  }

  /**
   * Returns a command that goes to a position
   * @param drive drivetrain to be used for the command
   * @param pose pose to go to
   * @param relativeToRobot should pose be converted to a transformation
   */
  public GoToPose(Drivetrain drive, Pose2d pose, boolean relativeToRobot) {
    this(drive, () -> pose, relativeToRobot);
  }

  /**
   * Returns a command that goes to a position based off the position supplier
   * @param drive drivetrain to be used for the command
   * @param poseSupplier supplier of pose go to
   * @param relativeToRobot should pose be converted to a transformation
   */
  public GoToPose(Drivetrain drive, Supplier<Pose2d> poseSupplier, boolean relativeToRobot) {
    m_drive = drive;
    addRequirements(drive);
    m_poseSupplier = poseSupplier;
    m_relativeToRobot = relativeToRobot;
  }
  
  @Override
  public void initialize() {
    m_finalPose = m_poseSupplier.get();
    if (m_relativeToRobot) m_finalPose = m_drive.getPose().plus(
      m_finalPose.minus(new Pose2d()) // the minus method is to make the pose a transform
    );
    m_startTime = Timer.getFPGATimestamp();
  }
  
  @Override
  public void execute() {
    m_drive.runChassisPID(m_finalPose.getX(), m_finalPose.getY(), m_finalPose.getRotation().getRadians()); 
  }

  @Override
  public boolean isFinished() {
    if (m_doNotEnd) return false;
    double errorMarginMeters = TestConstants.kTranslationError;
    double errorMarginRadians = TestConstants.kHeadingError;
    m_error = m_drive.getPose().relativeTo(m_finalPose);
    // if robot thinks its precision is < 0.1 to the target we inputted, it will stop, so then we can see how off it is
    return Math.abs(m_error.getX()) < errorMarginMeters &&
      Math.abs(m_error.getY()) < errorMarginMeters && 
      Math.abs(m_error.getRotation().getRadians()) < errorMarginRadians;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    System.out.println(Timer.getFPGATimestamp() - m_startTime);
    System.out.println(m_error.getX());
    System.out.println(m_error.getY());
    System.out.println(m_error.getRotation().getRadians());
  }
}

