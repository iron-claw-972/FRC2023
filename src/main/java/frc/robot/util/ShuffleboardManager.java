package frc.robot.util;


import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
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
  
  // hashmaps for storeing values corilateing to module
  Map<Module,Double> m_velModulesSaver=new HashMap<Module,Double>();
  Map<Module,Double> m_staticModulesSaver=new HashMap<Module,Double>();
  // modules needed to distigue in chooser
  Module m_dummyModule = Module.create(0, 0, 0, 0.0, 0.0, 0.0);
  Module m_allModule = Module.create(0, 0, 0, 0.0, 0.0, 0.0);
  // previous module for switching
  Module m_prevModule = m_dummyModule;
  
  // tabs
  ShuffleboardTab m_mainTab, m_drivetrainTab, m_swerveModulesTab, m_autoTab;

  // drive inputs
  GenericEntry m_heading;

  // swerve inputs
  GenericEntry m_driveVelocity, m_steerAngle, m_driveStaticFeedforward, m_driveVelocityFeedforward, m_drivetrainvolts;
  
  GenericEntry m_commandScheduler;
  
  // initilize choosers
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();
  SendableChooser<TestType> m_testMode = new SendableChooser<>();
  SendableChooser<Module> m_module = new SendableChooser<>();

  
  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    
    //shuffleboard tabs
    m_mainTab = Shuffleboard.getTab("Main");
    m_drivetrainTab = Shuffleboard.getTab("Drive");
    m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
    m_autoTab = Shuffleboard.getTab("Auto");
    
    // stores adds Commands sqeduler to shuffleboard
    m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();

    // set options to choosers
    autoChooserOptions();
    testTypeChooserOptions();
    moduleChooserOptions();
    // puting defult value in hashmaps so no null pointer errors oucur
    setUpFeedforwardHashmap();
    
    // add choosers
    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_mainTab.add("Practice Mode Type Chooser", m_testMode);
    m_swerveModulesTab.add("Module Chooser", m_module);

    // tab setup
    setupDrivetrain();
    setupModules();
  }

  private void setupDrivetrain() {
    // inputs
    m_heading = m_drivetrainTab.add("Set Heading (-pi to pi)", 0).getEntry();
    
    // add PID controlers
    m_drivetrainTab.add("xController", Robot.drive.getXController());
    m_drivetrainTab.add("yController", Robot.drive.getYController());
    m_drivetrainTab.add("rotationController", Robot.drive.getRotationController());

    m_drivetrainTab.addNumber("getAngle", () -> Robot.drive.getAngleHeading());
    m_drivetrainTab.addNumber("heading PID output", () -> Robot.drive.m_headingPIDOutput);

    m_drivetrainTab.addNumber("Gyro X", () -> Robot.drive.getAngularRate(0));
    m_drivetrainTab.addNumber("Gyro Y", () -> Robot.drive.getAngularRate(1));
    m_drivetrainTab.addNumber("Gyro Z", () -> Robot.drive.getAngularRate(2));
  }
  private void setupModules(){
    // inputs
    m_driveVelocity = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
    m_steerAngle = m_swerveModulesTab.add("Set Steer Angle", 0).getEntry();
    m_driveStaticFeedforward = m_swerveModulesTab.add("Set Drive Static Feedforward", 0).getEntry();
    m_driveVelocityFeedforward = m_swerveModulesTab.add("Set Drive Velocity Feedforward", 0).getEntry();
    m_drivetrainvolts = m_swerveModulesTab.add("Set Volts", 0).getEntry();
    
    // Desired Drive Velocitys
    // m_swerveModulesTab.addNumber("FL desired speed", () -> Robot.drive.swerveModuleStates[0].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("FR desired speed", () -> Robot.drive.swerveModuleStates[1].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BL desired speed", () -> Robot.drive.swerveModuleStates[2].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BR desired speed", () -> Robot.drive.swerveModuleStates[3].speedMetersPerSecond);

    // Desired Steer angles
    // m_swerveModulesTab.addNumber("FL desired angle", () -> Robot.drive.swerveModuleStates[0].angle.getDegrees());
    // m_swerveModulesTab.addNumber("FR desired angle", () -> Robot.drive.swerveModuleStates[1].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BL desired angle", () -> Robot.drive.swerveModuleStates[2].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BR desired angle", () -> Robot.drive.swerveModuleStates[3].angle.getDegrees());

    // Steer angles
    // m_swerveModulesTab.addNumber("Angle FL",  () -> Units.radiansToDegrees(Robot.drive.m_modules[0].getAngle()));
    // m_swerveModulesTab.addNumber("Angle FR", () -> Units.radiansToDegrees(Robot.drive.m_modules[1].getAngle()));
    // m_swerveModulesTab.addNumber("Angle BL",   () -> Units.radiansToDegrees(Robot.drive.m_modules[2].getAngle()));
    // m_swerveModulesTab.addNumber("Angle BR",  () -> Units.radiansToDegrees(Robot.drive.m_modules[3].getAngle()));

    // Drive PID output
    // m_swerveModulesTab.addNumber("FL PID Output", () -> Robot.drive.m_modules[0].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("FR PID Output", () -> Robot.drive.m_modules[1].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("BL PID Output", () -> Robot.drive.m_modules[2].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("BR PID Output", () -> Robot.drive.m_modules[3].getDrivePIDOutput());

    // get drive velocity
    m_swerveModulesTab.addNumber("Vel FL Raw", () -> Robot.drive.m_modules[0].getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel FR Raw", () -> Robot.drive.m_modules[1].getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel BL Raw", () -> Robot.drive.m_modules[2].getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel BR Raw", () -> Robot.drive.m_modules[3].getDriveVelocity());

    // drivePIDS
    m_swerveModulesTab.add("Drive PID FL", Robot.drive.m_modules[0].getDrivePID());
    m_swerveModulesTab.add("Drive PID FR", Robot.drive.m_modules[1].getDrivePID());
    m_swerveModulesTab.add("Drive PID BL", Robot.drive.m_modules[2].getDrivePID());
    m_swerveModulesTab.add("Drive PID BR", Robot.drive.m_modules[3].getDrivePID());

    //Median Filltered Velocity Values
    // m_swerveModulesTab.addNumber("Vel FL Filtered", () -> Robot.drive.m_modules[0].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel FR Filtered", () -> Robot.drive.m_modules[1].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel BL Filtered", () -> Robot.drive.m_modules[2].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel BR Filtered", () -> Robot.drive.m_modules[3].getDriveVelocityFilltered());
  }
  //puting defult value in hashmaps
  private void setUpFeedforwardHashmap(){
    m_staticModulesSaver.put(m_dummyModule,0.0);
    m_velModulesSaver.put(m_dummyModule,0.0);
    m_staticModulesSaver.put(m_allModule,Constants.drive.kDriveKSAll);
    m_velModulesSaver.put(m_allModule,Constants.drive.kDriveKVAll);
    m_staticModulesSaver.put(Robot.drive.m_modules[0],Constants.drive.kDriveKSFrontLeft);
    m_velModulesSaver.put(Robot.drive.m_modules[0],Constants.drive.kDriveKVFrontLeft);
    m_staticModulesSaver.put(Robot.drive.m_modules[1],Constants.drive.kDriveKSFrontRight);
    m_velModulesSaver.put(Robot.drive.m_modules[1],Constants.drive.kDriveKVFrontRight);
    m_staticModulesSaver.put(Robot.drive.m_modules[2],Constants.drive.kDriveKSBackLeft);
    m_velModulesSaver.put(Robot.drive.m_modules[2],Constants.drive.kDriveKVBackLeft);
    m_staticModulesSaver.put(Robot.drive.m_modules[3],Constants.drive.kDriveKSBackRight);
    m_velModulesSaver.put(Robot.drive.m_modules[3],Constants.drive.kDriveKVBackRight);
  }

  //add options to choosers
  public void autoChooserOptions() {
    m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
    // m_autoCommand.setDefaultOption("TestAuto", new PathPlannerCommand("TestAuto", 0)); 
  }
  public void testTypeChooserOptions() {
    m_testMode.addOption(TestType.HEADING_PID.toString(), TestType.HEADING_PID);
    m_testMode.addOption(TestType.MODULE_DRIVE_VELOCITY.toString(), TestType.MODULE_DRIVE_VELOCITY);
    m_testMode.addOption(TestType.MODULE_STEER_ANGLE.toString(), TestType.MODULE_STEER_ANGLE);
    m_testMode.addOption(TestType.DRIVE_VOLTAGE.toString(), TestType.DRIVE_VOLTAGE);
    m_testMode.addOption(TestType.STEER_VOLTAGE.toString(), TestType.STEER_VOLTAGE);
    m_testMode.setDefaultOption(TestType.NONE.toString(), TestType.NONE);
  }
  private void moduleChooserOptions(){
    m_module.setDefaultOption("NONE", m_dummyModule);
    m_module.addOption("Front Left", Robot.drive.m_modules[0]);
    m_module.addOption("Front Right", Robot.drive.m_modules[1]);
    m_module.addOption("Back Left ", Robot.drive.m_modules[2]);
    m_module.addOption("Back Right", Robot.drive.m_modules[3]);
    m_module.addOption("all", m_allModule);
  }

  public void loadCommandSchedulerShuffleboard() {
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish

    CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker("Command initialized", command.getName(), EventImportance.kNormal));

    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker("Command interrupted", command.getName(), EventImportance.kNormal));

    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker("Command finished", command.getName(), EventImportance.kNormal));
  }


  //getters
  public TestType getTestModeType() {
    return m_testMode.getSelected();
  }
  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }
  public double getRequestedHeading() {
    return m_heading.getDouble(0);
  }
  public double getRequestedVelocity() {
    return m_driveVelocity.getDouble(0);
  }
  public double getRequestedVolts(){
    return m_drivetrainvolts.getDouble(0);
  }
  public double getRequestedSteerAngle() {
    return m_steerAngle.getDouble(0);
  }
  public double getDriveStaticFeedforward() {
    return m_driveStaticFeedforward.getDouble(0);
  }
  public double getDriveVelocityFeedforward() {
    return m_driveVelocityFeedforward.getDouble(0);
  }

  public void setModulefeedforward(){
    //revert to previous saved feed forward data if changed
    if (m_prevModule != m_module.getSelected()){
      m_driveStaticFeedforward.setDouble(m_staticModulesSaver.get(m_module.getSelected()));
      m_driveVelocityFeedforward.setDouble(m_velModulesSaver.get(m_module.getSelected()));
      m_prevModule = m_module.getSelected();
    }

    
    // update saved feedforward data
    m_staticModulesSaver.replace(m_module.getSelected(),m_driveStaticFeedforward.getDouble(0) );
    m_velModulesSaver.replace(m_module.getSelected(),m_driveVelocityFeedforward.getDouble(0) );
    
    //to set all modules to same feedforward values if all
    if (m_module.getSelected() == m_allModule){
      for(int i = 0; i < 4; i++){
        Robot.drive.m_modules[i].getShuffleboardFeedForwardValues(m_staticModulesSaver.get(m_module.getSelected()), m_velModulesSaver.get(m_module.getSelected()));
      }
    }
    //set selected module
    m_module.getSelected().getShuffleboardFeedForwardValues(m_staticModulesSaver.get(m_module.getSelected()),m_velModulesSaver.get(m_module.getSelected()));
  }
}
