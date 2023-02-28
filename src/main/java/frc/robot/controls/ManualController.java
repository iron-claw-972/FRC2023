package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class ManualController {
  GameController m_manual = new GameController(OIConstants.kManualJoy);
  private FourBarArm m_arm;
  private Intake m_intake;
  private Elevator m_elevator;
  
  public ManualController(FourBarArm arm, Intake intake, Elevator elevator){
    m_arm = arm;
    m_intake = intake;
    m_elevator = elevator;
  }
  
  public void configureControls() {
    if(m_intake != null){
      m_manual.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed), m_intake));
      m_manual.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakeSpeed),m_intake));
      m_manual.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stop(), m_intake));  
    }

    if(m_arm != null){
      m_manual.get(Button.Y).onTrue(new InstantCommand(() -> m_arm.setMotorPower(0.05)));
      m_manual.get(Button.X).onTrue(new InstantCommand(() -> m_arm.setMotorPower(-0.05)));
      m_manual.get(Button.B).onTrue(new InstantCommand(() -> m_arm.setMotorPower(0)));  
    }
    
    if(m_elevator != null){

    }
    
  }
  public double getManualElevatorPower() {
    return Functions.deadband(m_manual.get(Axis.LEFT_Y), OIConstants.kDeadband);
  }
}