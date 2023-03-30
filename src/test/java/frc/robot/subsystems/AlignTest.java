package frc.robot.subsystems;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.GoToPose;
import frc.robot.constants.Constants;
import frc.robot.util.Node;

/**
 * Tests the robot alignment
 */
public class AlignTest {

  // get the shuffleboard tabs to construct the drivetrain
  static ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");
  static ShuffleboardTab m_modulesTab = Shuffleboard.getTab("Swerve Modules");

  Drivetrain m_drive;
  
  @BeforeEach
  public void prepare() {
    Constants.kUseTelemetry = false;
    m_drive = new Drivetrain(m_driveTab, m_modulesTab, null);
    m_drive.enableVision(false);
  }

  @Test
  public void testGridAlign() {
    m_drive.resetOdometry(new Pose2d(0.8, 1, new Rotation2d(Math.PI)));
    Pose2d scorePose = new Node(Alliance.Blue, 3, 2).scorePose;
    GoToPose command = new GoToPose(() -> scorePose, m_drive);
    command.initialize();
    for (int i = 0; i < 100; i++) {
      CommandScheduler.getInstance().run();
    }
  }

}
