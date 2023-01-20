package frc.robot.util;


import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Module;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();
  Map<Module,Double> m_velModulesSaver=new HashMap<Module,Double>();
  Map<Module,Double> m_staticModulesSaver=new HashMap<Module,Double>();
  Module m_dummyModule = Module.create(0, 0, 0, 0.0, 0.0, 0.0);
  Module m_allModule = Module.create(0, 0, 0, 0.0, 0.0, 0.0);
  Module m_prevModule = m_dummyModule;
  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  public ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");
  ShuffleboardTab m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  
  GenericEntry m_heading = m_driveTab.add("Set Heading (-pi to pi)", 0).getEntry();
  GenericEntry m_velocity = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
  GenericEntry m_turn = m_swerveModulesTab.add("Set Drive Turn", 0).getEntry();
  GenericEntry m_staticFeedforward = m_swerveModulesTab.add("Set Static Feedforward", 0).getEntry();
  GenericEntry m_velFeedforward = m_swerveModulesTab.add("Set Vel Feedforward", 0).getEntry();

  GenericEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();

  SendableChooser<TestType> m_testMode = new SendableChooser<>();
  SendableChooser<Module> m_module = new SendableChooser<>();

  
  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    autoChooserUpdate();
    testTypeChooserUpdate();
    moduleChooserSetup();
    
    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_mainTab.add("Practice Mode Type Chooser", m_testMode);
    m_swerveModulesTab.add("Module Feedforward", m_module);
    setupDrivetrain();

    m_driveTab.add("xController", Robot.drive.getXController());
    m_driveTab.add("yController", Robot.drive.getYController());
    m_driveTab.add("rotationController", Robot.drive.getRotationController());
    m_driveTab.addNumber("getAngle", () -> Robot.drive.getAngleHeading());
    m_driveTab.addNumber("heading PID output", () -> Robot.drive.m_headingPIDOutput);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  public void autoChooserUpdate() {
    m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
    // m_autoCommand.setDefaultOption("TestAuto", new PathPlannerCommand("TestAuto", 0)); 
  }
  public void moduleChooserSetup(){
    m_module.setDefaultOption("NONE", m_dummyModule);
    m_module.addOption("Front Left", Robot.drive.m_modules[0]);
    m_module.addOption("Front Right", Robot.drive.m_modules[1]);
    m_module.addOption("Back Left ", Robot.drive.m_modules[2]);
    m_module.addOption("Back Right", Robot.drive.m_modules[3]);
    m_module.addOption("all", m_allModule );

    m_staticModulesSaver.put(m_dummyModule,0.0);
    m_velModulesSaver.put(m_dummyModule,0.0);
    m_staticModulesSaver.put(m_allModule,Constants.drive.kDriveAllKS);
    m_velModulesSaver.put(m_allModule,Constants.drive.kDriveAllKV);
    m_staticModulesSaver.put(Robot.drive.m_modules[0],Constants.drive.kDriveAllKS);
    m_velModulesSaver.put(Robot.drive.m_modules[0],Constants.drive.kDriveAllKV);
    m_staticModulesSaver.put(Robot.drive.m_modules[1],Constants.drive.kDriveAllKS);
    m_velModulesSaver.put(Robot.drive.m_modules[1],Constants.drive.kDriveAllKV);
    m_staticModulesSaver.put(Robot.drive.m_modules[2],Constants.drive.kDriveAllKS);
    m_velModulesSaver.put(Robot.drive.m_modules[2],Constants.drive.kDriveAllKV);
    m_staticModulesSaver.put(Robot.drive.m_modules[3],Constants.drive.kDriveAllKS);
    m_velModulesSaver.put(Robot.drive.m_modules[3],Constants.drive.kDriveAllKV);

  }

  public void getModulefeedforward(){
    //revert to previous saved feed forward data
    if (m_prevModule != m_module.getSelected()){
      m_staticFeedforward.setDouble(m_staticModulesSaver.get(m_module.getSelected()));
      m_velFeedforward.setDouble(m_velModulesSaver.get(m_module.getSelected()));
      m_prevModule = m_module.getSelected();
    }
    //to set all modules to same feedforward values
    if (m_module.getSelected() == m_allModule){
      for(int i = 0; i < 4; i++){
        Robot.drive.m_modules[i].getShuffleboardFeedForwardValues(m_staticModulesSaver.get(m_module.getSelected()), m_velModulesSaver.get(m_module.getSelected()));
      }
    }  
    // update saved feedforward data
    m_staticModulesSaver.replace(m_module.getSelected(),m_staticFeedforward.getDouble(0) );
    m_velModulesSaver.replace(m_module.getSelected(),m_velFeedforward.getDouble(0) );
    
    m_module.getSelected().getShuffleboardFeedForwardValues(m_staticModulesSaver.get(m_module.getSelected()),m_velModulesSaver.get(m_module.getSelected()));
  }
  public TestType getTestModeType() {
    return m_testMode.getSelected();
  }

  public void testTypeChooserUpdate() {
    m_testMode.addOption(TestType.TUNE_HEADING_PID.toString(), TestType.TUNE_HEADING_PID);
    m_testMode.addOption(TestType.TUNE_MODULE_DRIVE.toString(), TestType.TUNE_MODULE_DRIVE);
    m_testMode.addOption(TestType.TUNE_MODULE_TURN.toString(), TestType.TUNE_MODULE_TURN);
    m_testMode.setDefaultOption(TestType.NONE.toString(), TestType.NONE);
  }

  public double getRequestedHeading() {
    return m_heading.getDouble(0);
  }

  public double getRequestedVelocity() {
    return m_velocity.getDouble(0);
  }

  public double getRequestedTurnAngle() {
    return m_turn.getDouble(0);
  }
  public double getStaticFeedforward() {
    return m_staticFeedforward.getDouble(0);
  }
  public double getVelocityFeedforward() {
    return m_velFeedforward.getDouble(0);
  }

  public void loadCommandSchedulerShuffleboard(){
    CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString(command.getName() + " initialized."));
    CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString(command.getName() + " interrupted."));
    CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
  }

  private void setupDrivetrain() {
    m_swerveModulesTab.addNumber("Angle Front Left",  () -> Units.radiansToDegrees(Robot.drive.m_modules[0].getAngle()));
    m_swerveModulesTab.addNumber("Angle Front Right", () -> Units.radiansToDegrees(Robot.drive.m_modules[1].getAngle()));
    m_swerveModulesTab.addNumber("Angle Back Left",   () -> Units.radiansToDegrees(Robot.drive.m_modules[2].getAngle()));
    m_swerveModulesTab.addNumber("Angle Back Right",  () -> Units.radiansToDegrees(Robot.drive.m_modules[3].getAngle()));

    m_driveTab.addNumber("Gyro X", () -> Robot.drive.getAngularRate(0));
    m_driveTab.addNumber("Gyro Y", () -> Robot.drive.getAngularRate(1));
    m_driveTab.addNumber("Gyro Z", () -> Robot.drive.getAngularRate(2));

    // m_swerveModulesTab.addNumber("FL desired speed", () -> Robot.drive.swerveModuleStates[0].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("FR desired speed", () -> Robot.drive.swerveModuleStates[1].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BL desired speed", () -> Robot.drive.swerveModuleStates[2].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BR desired speed", () -> Robot.drive.swerveModuleStates[3].speedMetersPerSecond);

    // m_swerveModulesTab.addNumber("FL desired angle", () -> Robot.drive.swerveModuleStates[0].angle.getDegrees());
    // m_swerveModulesTab.addNumber("FR desired angle", () -> Robot.drive.swerveModuleStates[1].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BL desired angle", () -> Robot.drive.swerveModuleStates[2].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BR desired angle", () -> Robot.drive.swerveModuleStates[3].angle.getDegrees());

    // m_swerveModulesTab.addNumber("FL FF", () -> Robot.drive.m_modules[0].getTurnFeedForward());
    // m_swerveModulesTab.addNumber("FR FF", () -> Robot.drive.m_modules[1].getTurnFeedForward());
    // m_swerveModulesTab.addNumber("BL FF", () -> Robot.drive.m_modules[2].getTurnFeedForward());
    // m_swerveModulesTab.addNumber("BR FF", () -> Robot.drive.m_modules[3].getTurnFeedForward());

    m_swerveModulesTab.addNumber("FL PID Output", () -> Robot.drive.m_modules[0].getSteerOutput());
    m_swerveModulesTab.addNumber("FR PID Output", () -> Robot.drive.m_modules[1].getSteerOutput());
    m_swerveModulesTab.addNumber("BL PID Output", () -> Robot.drive.m_modules[2].getSteerOutput());
    m_swerveModulesTab.addNumber("BR PID Output", () -> Robot.drive.m_modules[3].getSteerOutput());


    m_swerveModulesTab.addNumber("Vel Front Left", () -> Robot.drive.m_modules[0].getDriveVelocityFilltered());
    m_swerveModulesTab.addNumber("Vel Front Right", () -> Robot.drive.m_modules[1].getDriveVelocityFilltered());
    m_swerveModulesTab.addNumber("Vel Back Left", () -> Robot.drive.m_modules[2].getDriveVelocityFilltered());
    m_swerveModulesTab.addNumber("Vel Back Right", () -> Robot.drive.m_modules[3].getDriveVelocityFilltered());
  }

}
