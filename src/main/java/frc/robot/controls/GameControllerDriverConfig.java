package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  
  private GameController driverGC = new GameController(OIConstants.kDriverJoy);
  
  public GameControllerDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
    super(drive, controllerTab,shuffleboardUpdates);
  }
  
  public void configureControls() { 
    driverGC.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
    driverGC.get(Button.A).whileTrue(new SetFormationX(super.getDrivetrain()));
  }
  
  public double getRawSideTranslation() { 
    return driverGC.get(Axis.LEFT_X);
  }
  
  public double getRawForwardTranslation() {
    return driverGC.get(Axis.LEFT_Y);
  }
  public double getRawRotation() { 
    return driverGC.get(Axis.RIGHT_X);
  }
  
  public double getRawHeadingAngle() { 
    return Functions.calculateAngle(driverGC.get(Axis.RIGHT_X),-driverGC.get(Axis.RIGHT_Y))-Math.PI/2;
  }
  
  public double getRawHeadingMagnitude() { 
    return Functions.calculateHypotenuse(driverGC.get(Axis.RIGHT_X),driverGC.get(Axis.RIGHT_Y));
  }
  
}
