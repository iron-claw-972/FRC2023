package frc.robot.controls;

import frc.robot.commands.SetFormationX;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.Ex3DProController;
import lib.controllers.Ex3DProController.Ex3DProAxis;
import lib.controllers.Ex3DProController.Ex3DProButton;

public class Ex3DProControllerConfig extends BaseControllerConfig {

  private static Ex3DProController driverEPC = new Ex3DProController(OIConstants.kDriverJoy);
  private final Drivetrain m_drive = new Drivetrain();

  public void configureControls() { 
  driverEPC.get(Ex3DProButton.B1).whileTrue(new SetFormationX(m_drive));
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
