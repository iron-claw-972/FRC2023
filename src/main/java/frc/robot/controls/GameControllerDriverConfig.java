package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoToNode;
import frc.robot.commands.GoToNodePID;
import frc.robot.commands.GoToShelf;
import frc.robot.commands.GoToShelfPID;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  
  private final GameController kDriver = new GameController(OIConstants.kDriverJoy);
  private final Operator m_operator;
  public GameControllerDriverConfig(Drivetrain drive, Operator operator, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
    super(drive, controllerTab, shuffleboardUpdates);
    m_operator=operator;
  }
  
  @Override
  public void configureControls() { 
    kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
      new Rotation2d(DriverStation.getAlliance() == Alliance.Blue ? 0 : Math.PI)
    )));
    kDriver.get(Button.X).onTrue(new SetFormationX(super.getDrivetrain()));

    // Moves to the selected scoring position using Path Planner
    kDriver.get(Button.LB).whileTrue(new GoToNode(m_operator, getDrivetrain()));
    // Moves to the selected scoring position using the PID
    kDriver.get(DPad.LEFT).whileTrue(new GoToNodePID(m_operator, getDrivetrain()));

    // Moves to the shelf using Path Planner
    kDriver.get(Button.RB).whileTrue(new GoToShelf(getDrivetrain()));
    // Moves to the shelf using the PID
    kDriver.get(DPad.RIGHT).whileTrue(new GoToShelfPID(getDrivetrain()));
    
    kDriver.get(Button.B).whileTrue(new BalanceCommand(super.getDrivetrain()));

    kDriver.get(Button.A).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));
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
