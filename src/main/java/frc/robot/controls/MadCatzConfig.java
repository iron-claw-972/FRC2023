package frc.robot.controls;

import frc.robot.commands.SetFormationX;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.MadCatzController;
import lib.controllers.MadCatzController.MadCatzAxis;
import lib.controllers.MadCatzController.MadCatzButton;

public class MadCatzConfig extends BaseControllerConfig {

  private static MadCatzController driverMCC = new MadCatzController(OIConstants.kDriverJoy);
  private final Drivetrain m_drive = new Drivetrain();

  public void configureControls() { 
    driverMCC.get(MadCatzButton.B1).whileTrue(new SetFormationX(m_drive));
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




    
