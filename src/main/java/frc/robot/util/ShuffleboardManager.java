package frc.robot.util;


import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveFeedForwardCharacterzation;
import frc.robot.commands.SteerFeedForwardCharacterzation;
import frc.robot.commands.SteerFeedForwardCharacterzationAll;
import frc.robot.commands.auto.OdometryTestCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.DriveConstants.CompDriveConstants;
import frc.robot.constants.DriveConstants.TestDriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Module;
import lib.controllers.Controller;
import lib.controllers.Ex3DProController;
import lib.controllers.GameController;
import lib.controllers.MadCatzController;

public class ShuffleboardManager {
  
  // hashmaps for storeing values corilateing to module
  public Map<Module,Double> m_driveVelFeedForwardSaver=new HashMap<Module,Double>();
  public Map<Module,Double> m_driveStaticFeedForwardSaver=new HashMap<Module,Double>();
  public Map<Module,Double> m_steerVelFeedForwardSaver=new HashMap<Module,Double>();
  public Map<Module,Double> m_steerStaticFeedForwardSaver=new HashMap<Module,Double>();
  Boolean invert = false;
  // modules needed to distigue in chooser
  Module m_dummyModule = Module.create(ModuleConstants.NONE);
  Module m_allModule = Module.create(ModuleConstants.NONE);
  // previous module for switching
  Module m_prevModule = m_dummyModule;
  // tabs
  ShuffleboardTab m_mainTab, m_drivetrainTab, m_swerveModulesTab, m_autoTab, m_controllerTab;

  // drive inputs
  GenericEntry m_heading;

  // swerve inputs
  GenericEntry 
    m_driveVelocity,
    m_steerVelocity, 
    m_steerAngle, 
    m_drivetrainvolts, 
    m_driveStaticFeedforward, 
    m_driveVelocityFeedforward, 
    m_steerStaticFeedforward,
    m_steerVelocityFeedforward;
  
  //controller inputs
  GenericEntry m_translationalSenseitivity, m_translationalExpo, m_translationalDeadband, m_translationalSlewrate, m_fieldRelative;
  GenericEntry m_rotationSenseitiviy, m_rotationExpo, m_rotationDeadband, m_rotationSlewrate;
  GenericEntry m_headingSenseitiviy, m_headingExpo, m_headingDeadband;

  GenericEntry m_commandScheduler;
  
  // initilize choosers
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();
  SendableChooser<TestType> m_testMode = new SendableChooser<>();
  SendableChooser<Module> m_module = new SendableChooser<>();
  SendableChooser<Controller> m_controllerType = new SendableChooser<>();
  SendableChooser<RobotType> m_robotType = new SendableChooser<>();

  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    
    //shuffleboard tabs
    m_mainTab = Shuffleboard.getTab("Main");
    m_drivetrainTab = Shuffleboard.getTab("Drive");
    m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
    m_autoTab = Shuffleboard.getTab("Auto");
    m_controllerTab = Shuffleboard.getTab("Controller");
    
    // stores adds Commands sqeduler to shuffleboard
    m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();

    // set options to choosers
    autoChooserOptions();
    testTypeChooserOptions();
    moduleChooserOptions();
    controllerChooserOptions();
    // puting defult value in hashmaps so no null pointer errors oucur
    setUpFeedforwardHashmap();
    
    // add choosers
    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_mainTab.add("Practice Mode Type Chooser", m_testMode);
    m_mainTab.add("Robot Type Chooser", m_robotType);
    m_swerveModulesTab.add("Module Chooser", m_module);
    m_controllerTab.add("Controller Chooser", m_controllerType);

    // tab setup
    setupDrivetrain();
    setupModules();
    setupController();
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

    m_drivetrainTab.add(Robot.drive.getXController());
    m_drivetrainTab.add(Robot.drive.getYController());
    m_drivetrainTab.add(Robot.drive.getRotationController());
  }
  private void setupModules(){
    // inputs
    m_driveVelocity = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
    m_steerVelocity = m_swerveModulesTab.add("Set Steer Velocity", 0).getEntry();
    m_steerAngle = m_swerveModulesTab.add("Set Steer Angle", 0).getEntry();
    m_drivetrainvolts = m_swerveModulesTab.add("Set Volts", 0).getEntry();
    m_driveStaticFeedforward = m_swerveModulesTab.add("Drive kS FF", 0).getEntry();
    m_driveVelocityFeedforward = m_swerveModulesTab.add("Drive kV FF", 0).getEntry();
    m_steerStaticFeedforward = m_swerveModulesTab.add("Steer kS FF", 0).getEntry();
    m_steerVelocityFeedforward = m_swerveModulesTab.add("Steer kV k FF", 0).getEntry();
    
    // Desired Drive Velocitys
    // m_swerveModulesTab.addNumber("FL desired speed", () -> Robot.drive.swerveModuleStates[0].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("FR desired speed", () -> Robot.drive.swerveModuleStates[1].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BL desired speed", () -> Robot.drive.swerveModuleStates[2].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BR desired speed", () -> Robot.drive.swerveModuleStates[3].speedMetersPerSecond);

    // Drive PID output
    // m_swerveModulesTab.addNumber("FL PID Output", () -> Robot.drive.m_modules[0].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("FR PID Output", () -> Robot.drive.m_modules[1].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("BL PID Output", () -> Robot.drive.m_modules[2].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("BR PID Output", () -> Robot.drive.m_modules[3].getDrivePIDOutput());

    // get drive velocity
    // m_swerveModulesTab.addNumber("Vel FL Raw", () -> Robot.drive.m_modules[0].getDriveVelocity());
    // m_swerveModulesTab.addNumber("Vel FR Raw", () -> Robot.drive.m_modules[1].getDriveVelocity());
    // m_swerveModulesTab.addNumber("Vel BL Raw", () -> Robot.drive.m_modules[2].getDriveVelocity());
    // m_swerveModulesTab.addNumber("Vel BR Raw", () -> Robot.drive.m_modules[3].getDriveVelocity());

    // drivePIDS
    // m_swerveModulesTab.add("Drive PID FL", Robot.drive.m_modules[0].getDrivePID());
    // m_swerveModulesTab.add("Drive PID FR", Robot.drive.m_modules[1].getDrivePID());
    // m_swerveModulesTab.add("Drive PID BL", Robot.drive.m_modules[2].getDrivePID());
    // m_swerveModulesTab.add("Drive PID BR", Robot.drive.m_modules[3].getDrivePID());

    //Median Filltered Velocity Values
    // m_swerveModulesTab.addNumber("Vel FL Filtered", () -> Robot.drive.m_modules[0].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel FR Filtered", () -> Robot.drive.m_modules[1].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel BL Filtered", () -> Robot.drive.m_modules[2].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel BR Filtered", () -> Robot.drive.m_modules[3].getDriveVelocityFilltered());

    // Desired Steer angles
    // m_swerveModulesTab.addNumber("FL desired angle", () -> Robot.drive.m_swerveModuleStates[0].angle.getDegrees());
    // m_swerveModulesTab.addNumber("FR desired angle", () -> Robot.drive.m_swerveModuleStates[1].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BL desired angle", () -> Robot.drive.m_swerveModuleStates[2].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BR desired angle", () -> Robot.drive.m_swerveModuleStates[3].angle.getDegrees());

    // Steer angles
    m_swerveModulesTab.addNumber("Angle FL", () -> Robot.drive.m_modules[0].getAngle());
    m_swerveModulesTab.addNumber("Angle FR", () -> Robot.drive.m_modules[1].getAngle());
    m_swerveModulesTab.addNumber("Angle BL", () -> Robot.drive.m_modules[2].getAngle());
    m_swerveModulesTab.addNumber("Angle BR", () -> Robot.drive.m_modules[3].getAngle());

    // Steer Velocity
    m_swerveModulesTab.addNumber("Steer Vel FL", () -> Robot.drive.m_modules[0].getSteerVelocity());
    m_swerveModulesTab.addNumber("Steer Vel FR", () -> Robot.drive.m_modules[1].getSteerVelocity());
    m_swerveModulesTab.addNumber("Steer Vel BL", () -> Robot.drive.m_modules[2].getSteerVelocity());
    m_swerveModulesTab.addNumber("Steer Vel BR", () -> Robot.drive.m_modules[3].getSteerVelocity());

    //Steer PID
    m_swerveModulesTab.add("Steer PID FL", Robot.drive.m_modules[0].getSteerPID());
    m_swerveModulesTab.add("Steer PID FR", Robot.drive.m_modules[1].getSteerPID());
    m_swerveModulesTab.add("Steer PID BL", Robot.drive.m_modules[2].getSteerPID());
    m_swerveModulesTab.add("Steer PID BR", Robot.drive.m_modules[3].getSteerPID());
  }
  private void setupController(){

    m_controllerTab.add("Controller Type", m_controllerType);

    m_translationalSenseitivity = m_controllerTab.add("translationalSenseitivity", Constants.oi.kTranslationalSenseitivity).getEntry();
    m_translationalExpo = m_controllerTab.add("translationalExpo", Constants.oi.kTranslationalExpo).getEntry();
    m_translationalDeadband = m_controllerTab.add("translationalDeadband", Constants.oi.kTranslationalDeadband).getEntry();
    m_translationalSlewrate = m_controllerTab.add("translationalSlewrate", Constants.oi.kTranslationalSlewrate).getEntry();
    m_fieldRelative = m_controllerTab.add("Field Relitive", Constants.oi.kFieldRelative).getEntry();

    m_rotationSenseitiviy = m_controllerTab.add("rotationSenseitiviy", Constants.oi.kRotationSenseitiviy).getEntry();
    m_rotationExpo = m_controllerTab.add("rotationExpo", Constants.oi.kRotationExpo).getEntry();
    m_rotationDeadband = m_controllerTab.add("rotationDeadband", Constants.oi.kRotationDeadband).getEntry();
    m_rotationSlewrate = m_controllerTab.add("rotationSlewrate", Constants.oi.kRotationSlewrate).getEntry();

    m_headingSenseitiviy = m_controllerTab.add("headingSenseitiviy", Constants.oi.kHeadingSenseitiviy).getEntry();
    m_headingExpo = m_controllerTab.add("headingExpo", Constants.oi.kHeadingExpo).getEntry();
    m_headingDeadband = m_controllerTab.add("headingDeadband", Constants.oi.kHeadingDeadband).getEntry();


  }
  //puting defult value in hashmaps
  private void setUpFeedforwardHashmap(){
    m_driveStaticFeedForwardSaver.put(m_dummyModule,0.0);
    m_driveVelFeedForwardSaver.put(m_allModule,Constants.drive.kDriveKVAll);
    m_driveStaticFeedForwardSaver.put(Robot.drive.m_modules[0],TestDriveConstants.kDriveKSFrontLeft);
    m_driveStaticFeedForwardSaver.put(Robot.drive.m_modules[1],TestDriveConstants.kDriveKSFrontRight);
    m_driveStaticFeedForwardSaver.put(Robot.drive.m_modules[2],TestDriveConstants.kDriveKSBackLeft);
    m_driveStaticFeedForwardSaver.put(Robot.drive.m_modules[3],TestDriveConstants.kDriveKSBackRight);
    
    m_driveVelFeedForwardSaver.put(m_dummyModule,0.0);
    m_driveStaticFeedForwardSaver.put(m_allModule,Constants.drive.kDriveKSAll);
    m_driveVelFeedForwardSaver.put(Robot.drive.m_modules[0],TestDriveConstants.kDriveKVFrontLeft);
    m_driveVelFeedForwardSaver.put(Robot.drive.m_modules[1],TestDriveConstants.kDriveKVFrontRight);
    m_driveVelFeedForwardSaver.put(Robot.drive.m_modules[2],TestDriveConstants.kDriveKVBackLeft);
    m_driveVelFeedForwardSaver.put(Robot.drive.m_modules[3],TestDriveConstants.kDriveKVBackRight);
    

    m_steerStaticFeedForwardSaver.put(m_dummyModule,0.0);
    m_steerVelFeedForwardSaver.put(m_allModule,Constants.drive.kDriveKVAll);
    m_steerStaticFeedForwardSaver.put(Robot.drive.m_modules[0],TestDriveConstants.kDriveKSFrontLeft);
    m_steerStaticFeedForwardSaver.put(Robot.drive.m_modules[1],TestDriveConstants.kDriveKSFrontRight);
    m_steerStaticFeedForwardSaver.put(Robot.drive.m_modules[2],TestDriveConstants.kDriveKSBackLeft);
    m_steerStaticFeedForwardSaver.put(Robot.drive.m_modules[3],TestDriveConstants.kDriveKSBackRight);
    
    m_steerVelFeedForwardSaver.put(m_dummyModule,0.0);
    m_steerStaticFeedForwardSaver.put(m_allModule,Constants.drive.kDriveKSAll);
    m_steerVelFeedForwardSaver.put(Robot.drive.m_modules[0],TestDriveConstants.kDriveKVFrontLeft);
    m_steerVelFeedForwardSaver.put(Robot.drive.m_modules[1],TestDriveConstants.kDriveKVFrontRight);
    m_steerVelFeedForwardSaver.put(Robot.drive.m_modules[2],TestDriveConstants.kDriveKVBackLeft);
    m_steerVelFeedForwardSaver.put(Robot.drive.m_modules[3],TestDriveConstants.kDriveKVBackRight);
  }

  //add options to choosers
  public void autoChooserOptions() {
    m_autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
    m_autoCommand.addOption("Steer All FF charecterzation", new SteerFeedForwardCharacterzationAll(Robot.drive));
    m_autoCommand.addOption("Steer FF charecterzation", new SteerFeedForwardCharacterzation(Robot.drive));
    m_autoCommand.addOption("Self FF charecterzation", new DriveFeedForwardCharacterzation(Robot.drive));
    m_autoCommand.addOption("Odometry PID Test", new OdometryTestCommand(Robot.drive, new Transform2d(new Translation2d(1,1), new Rotation2d(Math.PI / 2))));

    // m_autoCommand.setDefaultOption("TestAuto", new PathPlannerCommand("TestAuto", 0)); 
  }

  public void robotTypeOptions() {
    m_robotType.setDefaultOption("Competition Robot", RobotType.COMP);
    m_robotType.addOption("Test Robot", RobotType.TEST);
  }

  public void testTypeChooserOptions() {
    m_testMode.addOption(TestType.HEADING_DRIVE.toString(), TestType.HEADING_DRIVE);
    m_testMode.addOption(TestType.HEADING_PID.toString(), TestType.HEADING_PID);
    m_testMode.addOption(TestType.MODULE_DRIVE_VELOCITY.toString(), TestType.MODULE_DRIVE_VELOCITY);
    m_testMode.addOption(TestType.MODULE_STEER_ANGLE.toString(), TestType.MODULE_STEER_ANGLE);
    m_testMode.addOption(TestType.DRIVE_VOLTAGE.toString(), TestType.DRIVE_VOLTAGE);
    m_testMode.addOption(TestType.STEER_VOLTAGE.toString(), TestType.STEER_VOLTAGE);
    m_testMode.addOption(TestType.ODOMETRY_TEST.toString(), TestType.ODOMETRY_TEST);
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
  private void controllerChooserOptions(){
    m_controllerType.setDefaultOption("GameController", new GameController(-1));
    m_controllerType.addOption("Ex3DPro", new Ex3DProController(-1));
    m_controllerType.addOption("MadCatz", new MadCatzController(-1));
    
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
  public double getRequestedDriveVelocity() {
    return m_driveVelocity.getDouble(0);
  }
  public double getRequestedSteerVelocity() {
    return m_steerVelocity.getDouble(0);
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
  
  // controller settings
  public double getTranslationalSenseitivity(){
    return m_translationalSenseitivity.getDouble(Constants.oi.kTranslationalSenseitivity);
  }
  public double getTranslationalExpo(){
    return m_translationalExpo.getDouble(Constants.oi.kTranslationalExpo);
  }
  public double getTranslationalDeadband(){
    return m_translationalDeadband.getDouble(Constants.oi.kTranslationalDeadband);
  }
  public double getTranslationalSlewrate(){
    return m_translationalSlewrate.getDouble(Constants.oi.kTranslationalSlewrate);
  }
  public boolean getFieldRelative(){
    return m_fieldRelative.getBoolean(Constants.oi.kFieldRelative);
  }
  public double getRotationSenseitiviy(){
    return m_rotationSenseitiviy.getDouble(Constants.oi.kRotationSenseitiviy);
  }
  public double getRotationExpo(){
    return m_rotationExpo.getDouble(Constants.oi.kRotationExpo);
  }
  public double getRotationDeadband(){
    return m_rotationDeadband.getDouble(Constants.oi.kRotationDeadband);
  }
  public double getRotationSlewrate(){
    return m_rotationSlewrate.getDouble(Constants.oi.kRotationSlewrate);
  }
  public double getHeadingSenseitiviy(){
    return m_headingSenseitiviy.getDouble(Constants.oi.kHeadingSenseitiviy);
  }
  public double getHeadingExpo(){
    return m_headingExpo.getDouble(Constants.oi.kHeadingExpo);
  }
  public double getHeadingDeadband(){
  return m_headingDeadband.getDouble(Constants.oi.kHeadingDeadband);
  }
  public Controller getControllerType(){
    return m_controllerType.getSelected();
  }

public RobotType getRobotType() {
  return m_robotType.getSelected();
}
public Boolean getInverted(){
  return invert;
}
public Module getModule(){
  return m_module.getSelected();
}

  public void setDriveModuleFeedforward(){
    //revert to previous saved feed forward data if changed
    
    if (m_prevModule != m_module.getSelected()){
      m_driveStaticFeedforward.setDouble(m_driveStaticFeedForwardSaver.get(m_module.getSelected()));
      m_driveVelocityFeedforward.setDouble(m_driveVelFeedForwardSaver.get(m_module.getSelected()));
      m_prevModule = m_module.getSelected();
    }
    
    // update saved feedforward data
    m_driveStaticFeedForwardSaver.replace(m_module.getSelected(),m_driveStaticFeedforward.getDouble(0) );
    m_driveVelFeedForwardSaver.replace(m_module.getSelected(),m_driveVelocityFeedforward.getDouble(0) );
    
    //to set all modules to same feedforward values if all
    if (m_module.getSelected() == m_allModule){
      for(int i = 0; i < 4; i++){
        Robot.drive.m_modules[i].setDriveFeedForwardValues(m_driveStaticFeedForwardSaver.get(m_module.getSelected()), m_driveVelFeedForwardSaver.get(m_module.getSelected()));
      }
    }
    //set selected module
    m_module.getSelected().setDriveFeedForwardValues(m_driveStaticFeedForwardSaver.get(m_module.getSelected()),m_driveVelFeedForwardSaver.get(m_module.getSelected()));
  }

  public void setSteerModuleFeedforward(){
    //revert to previous saved feed forward data if changed
    
    if (m_prevModule != m_module.getSelected()){
      m_steerStaticFeedforward.setDouble(m_steerStaticFeedForwardSaver.get(m_module.getSelected()));
      m_steerVelocityFeedforward.setDouble(m_steerVelFeedForwardSaver.get(m_module.getSelected()));
      m_prevModule = m_module.getSelected();
    }
    
    // update saved feedforward data
    m_steerStaticFeedForwardSaver.replace(m_module.getSelected(),m_steerStaticFeedforward.getDouble(0) );
    m_steerVelFeedForwardSaver.replace(m_module.getSelected(),m_steerVelocityFeedforward.getDouble(0) );
    
    //to set all modules to same feedforward values if all
    if (m_module.getSelected() == m_allModule){
      for(int i = 0; i < 4; i++){
        Robot.drive.m_modules[i].setDriveFeedForwardValues(m_steerStaticFeedForwardSaver.get(m_module.getSelected()), m_steerVelFeedForwardSaver.get(m_module.getSelected()));
      }
    }
    //set selected module
    m_module.getSelected().setDriveFeedForwardValues(m_steerStaticFeedForwardSaver.get(m_module.getSelected()),m_steerVelFeedForwardSaver.get(m_module.getSelected()));
  }
}
