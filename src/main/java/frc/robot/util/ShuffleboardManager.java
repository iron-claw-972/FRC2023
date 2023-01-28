package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Robot.Teams;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();
  SendableChooser<Teams> m_team = new SendableChooser<>();
  SendableChooser<TestType> m_testType = new SendableChooser<>();

  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  ShuffleboardTab m_testTab = Shuffleboard.getTab("Test");

  GenericEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();

  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    autoChooserUpdate();
    setupTeamChooser();

    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_testTab.add("Run Test", m_testType);
    m_mainTab.add("Alliance Chooser", m_team);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }
  public Teams getTeam() {
    return m_team.getSelected();
  }

  public void autoChooserUpdate() {
    m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
  }
  public void setupTeamChooser() {
    m_team.addOption("Blue", Teams.BLUE);
    m_team.addOption("Red", Teams.RED);
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
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish

    CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker("Command initialized", command.getName(), EventImportance.kNormal));

    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker("Command interrupted", command.getName(), EventImportance.kNormal));

    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker("Command finished", command.getName(), EventImportance.kNormal));
  }
}
