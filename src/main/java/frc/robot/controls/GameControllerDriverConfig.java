package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.BalanceCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.OIConstants;
import frc.robot.constants.swerve.DriveConstants;
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
  
  public GameControllerDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
    super(drive, controllerTab, shuffleboardUpdates);
  }

  public Trigger isSlowSpeed() {
    return kDriver.get(kDriver.RIGHT_TRIGGER_BUTTON);
  }
  
  @Override
  public void configureControls() { 
    kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
    kDriver.get(Button.A).whileTrue(new SetFormationX(super.getDrivetrain()));

    kDriver.get(Button.RB).onTrue(new BalanceCommand(super.getDrivetrain()));
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
    return Functions.calculateAngle(kDriver.get(Axis.RIGHT_X), -kDriver.get(Axis.RIGHT_Y)) - Math.PI/2;
  }
  
  @Override
  public double getRawHeadingMagnitude() { 
    return Functions.calculateHypotenuse(kDriver.get(Axis.RIGHT_X), kDriver.get(Axis.RIGHT_Y));
  }
  
}
