package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.GCAxis;
import lib.controllers.GameController.GCButton;

public class GameControllerDriverConfig extends BaseDriverConfig {

  private GameController driverGC = new GameController(OIConstants.kDriverJoy);

  public GameControllerDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates){
    super(drive, controllerTab,shuffleboardUpdates);
  }

  public void configureControls() { 
    driverGC.get(GCButton.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
    driverGC.get(GCButton.A).whileTrue(new SetFormationX(super.getDrivetrain()));
  }

  public double getRawSideTranslation() { 
    return driverGC.get(GCAxis.LEFT_X);
  }

  public double getRawForwardTranslation() {
    return driverGC.get(GCAxis.LEFT_Y);
  }
  public double getRawRotation() { 
    return driverGC.get(GCAxis.RIGHT_X);
  }

  public double getRawHeadingAngle() { 
    return Functions.calculateAngle(driverGC.get(GCAxis.RIGHT_X),-driverGC.get(GCAxis.RIGHT_Y))-Math.PI/2;
  }

  public double getRawHeadingMagnitude() { 
    return Functions.calculateHypotenuse(driverGC.get(GCAxis.RIGHT_X),driverGC.get(GCAxis.RIGHT_Y));
  }
     
}
