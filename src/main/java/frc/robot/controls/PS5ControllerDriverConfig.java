package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.GoToPose;
import frc.robot.commands.SetFormationX;
import frc.robot.commands.auto.BalanceCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
  
  private final PS5Controller kDriver = new PS5Controller(OIConstants.kDriverJoy);
  private final Operator m_operator;

  public PS5ControllerDriverConfig(Drivetrain drive, DoubleSupplier intakeOffset, Operator operator, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
    super(drive, intakeOffset, controllerTab, shuffleboardUpdates);
    m_operator = operator;
  }
  
  @Override
  public void configureControls() { 

    // reset the yaw forward. Mainly useful for testing/driver practice 
    kDriver.get(PS5Button.OPTIONS).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
      new Rotation2d(DriverStation.getAlliance() == Alliance.Blue ? 0 : Math.PI)
    )));

    // reset the yaw backward. Mainly useful for testing/driver practice 
    kDriver.get(PS5Button.CREATE).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
      new Rotation2d(DriverStation.getAlliance() == Alliance.Red ? 0 : Math.PI)
    )));

    // set the wheels to X
    kDriver.get(PS5Button.SQUARE).onTrue(new SetFormationX(super.getDrivetrain()));

    // Moves to the selected scoring position using Path Planner
    kDriver.get(PS5Button.TRIANGLE).whileTrue(new GoToPose(() -> getNodePose(), getDrivetrain()));

    // Moves to the single substation using Path Planner
    kDriver.get(PS5Button.RB).whileTrue(new GoToPose(() -> new Pose2d(
      // the single substation X
      DriverStation.getAlliance() == Alliance.Blue ? FieldConstants.kBlueSingleSubstationX : FieldConstants.kRedSingleSubstationX, 
      // don't move the robot in the Y
      getDrivetrain().getPose().getY(),
      // facing upwards
      new Rotation2d(Math.PI/2)),
      
      // max speed
      AutoConstants.kMaxAutoSpeed, 
      // max accel
      AutoConstants.kMaxAutoAccel + 1,
      
      getDrivetrain())
    );

    // Moves to the shelf using Path Planner
    kDriver.get(PS5Button.LB).whileTrue(new GoToPose(() -> (
      DriverStation.getAlliance() == Alliance.Blue ? VisionConstants.kBlueShelfAlignPose : VisionConstants.kRedShelfAlignPose), 
      getDrivetrain())
    );

    // Balances the robot
    kDriver.get(PS5Button.CIRCLE).whileTrue(new BalanceCommand(super.getDrivetrain()));

    // Resets the modules to absolute if they are having the unresolved zeroing error
    kDriver.get(PS5Button.CROSS).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));
  }

  public Pose2d getNodePose() {
    // get the desired score pose
    Pose2d scorePose = m_operator.getSelectedNode().scorePose;

    // get a y offset from the supplier (currently we use the intake to offset scoring)
    double yOffset = getIntakeOffset().getAsDouble();

    // modify pose by offsets
    scorePose = scorePose.plus(new Transform2d(
      new Translation2d(0, -0.07 + yOffset), 
      new Rotation2d(0)
    ));

    return scorePose;
  }
  
  @Override
  public double getRawSideTranslation() { 
    return kDriver.get(PS5Axis.LEFT_X);
  }
  
  @Override
  public double getRawForwardTranslation() {
    return kDriver.get(PS5Axis.LEFT_Y);
  }
  
  @Override
  public double getRawRotation() { 
    return kDriver.get(PS5Axis.RIGHT_X);
  }
  
  @Override
  public double getRawHeadingAngle() { 
    return Math.atan2(kDriver.get(PS5Axis.RIGHT_X), -kDriver.get(PS5Axis.RIGHT_Y)) - Math.PI/2;
  }
  
  @Override
  public double getRawHeadingMagnitude() { 
    return Functions.calculateHypotenuse(kDriver.get(PS5Axis.RIGHT_X), kDriver.get(PS5Axis.RIGHT_Y));
  }

  @Override
  public boolean getIsSlowMode() {
    return kDriver.get(PS5Button.RIGHT_TRIGGER).getAsBoolean();
  }

  @Override
  public boolean getIsAlign() {
    return kDriver.get(PS5Button.LEFT_TRIGGER).getAsBoolean();
  }
}
