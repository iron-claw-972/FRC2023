package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();
  SendableChooser<TestType> m_testType = new SendableChooser<>();

  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  ShuffleboardTab m_testTab = Shuffleboard.getTab("Test");

  NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();

  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    autoChooserUpdate();

    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_testTab.add("Run Test", m_testType);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  public void autoChooserUpdate() {
    m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
  }


  public TestType getTestType() {
    if (Robot.isTestMode()) {
      return m_testType.getSelected();
    }
    return TestType.NONE;
  }

  public Trigger isTestTypeTrigger(TestType testType) {
    return new Trigger(() -> m_testType.getSelected() == testType);
  }

  public void testChooserUpdate() {
    m_testType.addOption("No Test", TestType.NONE);
  }

  public void loadCommandSchedulerShuffleboard() {
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> m_commandScheduler.setString(command.getName() + " initialized."));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> m_commandScheduler.setString(command.getName() + " interrupted."));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
  }
}
