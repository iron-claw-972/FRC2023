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
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoToPose;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.OIConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  
  private final GameController kDriver = new GameController(OIConstants.kDriverJoy);
  private final Operator m_operator;
  public GameControllerDriverConfig(Drivetrain drive, DoubleSupplier intakeOffset, Operator operator, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
    super(drive, intakeOffset, controllerTab, shuffleboardUpdates);
    m_operator = operator;
  }
  
  @Override
  public void configureControls() { 
    kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
      new Rotation2d(DriverStation.getAlliance() == Alliance.Blue ? 0 : Math.PI)
    )));
    kDriver.get(Button.X).onTrue(new SetFormationX(super.getDrivetrain()));

    // Moves to the selected scoring position using Path Planner
    kDriver.get(Button.LB).whileTrue(new GoToPose(()->getNodePose(), getDrivetrain()));

    // Moves to the shelf using Path Planner
    kDriver.get(Button.RB).whileTrue(new GoToPose(()->(DriverStation.getAlliance()==Alliance.Blue?VisionConstants.kBlueShelfAlignPose:VisionConstants.kRedShelfAlignPose), getDrivetrain()));
    
    kDriver.get(Button.B).whileTrue(new BalanceCommand(super.getDrivetrain()));

    kDriver.get(Button.A).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));
  }

  public Pose2d getNodePose(){
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
    return kDriver.get(Axis.LEFT_X);
  }
  
  @Override
  public double getRawForwardTranslation() {
    return kDriver.get(Axis.LEFT_Y);
  }
  
  @Override
  public double getRawRotation() { 
    return kDriver.get(Axis.RIGHT_X);
  }
  
  @Override
  public double getRawHeadingAngle() { 
    return Math.atan2(kDriver.get(Axis.RIGHT_X), -kDriver.get(Axis.RIGHT_Y)) - Math.PI/2;
  }
  
  @Override
  public double getRawHeadingMagnitude() { 
    return Functions.calculateHypotenuse(kDriver.get(Axis.RIGHT_X), kDriver.get(Axis.RIGHT_Y));
  }

  @Override
  public boolean getIsSlowMode() {
    return kDriver.RIGHT_TRIGGER_BUTTON.getAsBoolean();
  }

  @Override
  public boolean getIsAlign() {
    return kDriver.LEFT_TRIGGER_BUTTON.getAsBoolean();
  }
}
