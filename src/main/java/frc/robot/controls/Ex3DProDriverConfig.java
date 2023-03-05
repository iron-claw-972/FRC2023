package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.GoToNode;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.OIConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.Ex3DProController;
import lib.controllers.Ex3DProController.Ex3DProAxis;
import lib.controllers.Ex3DProController.Ex3DProButton;

/**
 * Driver controls for the Ex3D Pro controller.
 */
public class Ex3DProDriverConfig extends BaseDriverConfig {
  
  private final Ex3DProController kDriver = new Ex3DProController(OIConstants.kDriverJoy);
  
  public Ex3DProDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
    super(drive, controllerTab, shuffleboardUpdates);
  }
  
  @Override
  public void configureControls() { 
    kDriver.get(Ex3DProButton.B1).whileTrue(new SetFormationX(super.getDrivetrain()));
    kDriver.get(Ex3DProButton.B2).onTrue(new InstantCommand(() -> super.getDrivetrain().setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
  }

  @Override
  public void configureControls(Operator operator) { 
    kDriver.get(Ex3DProButton.B3).whileTrue(new GoToNode(operator, getDrivetrain()));
  }

  @Override
  public double getRawSideTranslation() { 
    return -kDriver.get(Ex3DProAxis.X);     
  }
  
  @Override
  public double getRawForwardTranslation() {
    return -kDriver.get(Ex3DProAxis.Y);
  }
  
  @Override
  public double getRawRotation() { 
    return kDriver.get(Ex3DProAxis.Z);      
  }
  
  @Override
  public double getRawHeadingAngle() { 
    return kDriver.get(Ex3DProAxis.Z) * Math.PI;
  }
  
  @Override
  public double getRawHeadingMagnitude() { 
    return kDriver.get(Ex3DProAxis.SLIDER);
  }

  @Override
  public boolean getIsSlowMode() {
    return kDriver.get(Ex3DProButton.B11).getAsBoolean();
  }

  @Override
  public boolean getIsFieldRelative() {
    return !kDriver.get(Ex3DProButton.B12).getAsBoolean();
  }
}
