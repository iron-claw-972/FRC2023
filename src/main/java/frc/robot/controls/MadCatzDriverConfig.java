package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.MadCatzController;
import lib.controllers.MadCatzController.MadCatzAxis;
import lib.controllers.MadCatzController.MadCatzButton;

public class MadCatzDriverConfig extends BaseDriverConfig {

  private MadCatzController driverMCC = new MadCatzController(OIConstants.kDriverJoy);

  public MadCatzDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab){
    super(drive, controllerTab);
  }

  public void configureControls() { 
    driverMCC.get(MadCatzButton.B1).whileTrue(new SetFormationX(super.getDrivetrain()));
    driverMCC.get(MadCatzButton.B2).onTrue(new InstantCommand(() -> super.getDrivetrain().setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
  }

  public double getRawSideTranslation() { 
    return driverMCC.get(MadCatzAxis.X);
  }

  public double getRawForwardTranslation() {
    return -driverMCC.get(MadCatzAxis.Y);
  }
  public double getRawRotation() { 
    return driverMCC.get(MadCatzAxis.ZROTATE);
  }

  public double getRawHeadingAngle() { 
    return driverMCC.get(MadCatzAxis.ZROTATE) * Math.PI;
  }

  public double getRawHeadingMagnitude() { 
    return driverMCC.get(MadCatzAxis.SLIDER);
  }
      
}




    
