package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.Ex3DProController;
import lib.controllers.Ex3DProController.Ex3DProAxis;
import lib.controllers.Ex3DProController.Ex3DProButton;

public class Ex3DProDriverConfig extends BaseDriverConfig {

  private Ex3DProController driverEPC = new Ex3DProController(OIConstants.kDriverJoy);
  
  public Ex3DProDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates){
    super(drive, controllerTab, shuffleboardUpdates);
  }

  public void configureControls() { 
  driverEPC.get(Ex3DProButton.B1).whileTrue(new SetFormationX(super.getDrivetrain()));
  driverEPC.get(Ex3DProButton.B2).onTrue(new InstantCommand(() -> super.getDrivetrain().setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
  }

  public double getRawSideTranslation() { 
  return -driverEPC.get(Ex3DProAxis.X);     
  }

  public double getRawForwardTranslation() {
  return -driverEPC.get(Ex3DProAxis.Y);
  }

  public double getRawRotation() { 
  return driverEPC.get(Ex3DProAxis.Z);      
  }

  public double getRawHeadingAngle() { 
  return driverEPC.get(Ex3DProAxis.Z) * Math.PI;
  }

  public double getRawHeadingMagnitude() { 
  return driverEPC.get(Ex3DProAxis.SLIDER);
  }

}
