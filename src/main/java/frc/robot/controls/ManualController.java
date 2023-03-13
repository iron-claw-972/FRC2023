package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorMode;
import frc.robot.subsystems.RollerIntake;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class ManualController {
  GameController m_manual = new GameController(OIConstants.kManualJoy);
  private RollerIntake m_intake;
  private Elevator m_elevator;
  
  public ManualController(RollerIntake intake, Elevator elevator) {
    m_intake = intake;
    m_elevator = elevator;
  }
  
  public void configureControls() {
    if (m_intake != null) {
      // Commented out due to outdated code
      // TODO: Update to new intake commands
      // m_manual.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakePower), m_intake));
      // m_manual.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakePower),m_intake));
      // m_manual.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stopIntake(), m_intake));  
    }
    
    if (m_elevator != null) {
      m_manual.get(Button.RB).onTrue(new FunctionalCommand(
        () -> m_elevator.setMode(ElevatorMode.MANUAL), 
        () -> m_elevator.setDesiredPower(m_manual.get(Axis.LEFT_Y)), 
        interupted -> m_elevator.setMode(ElevatorMode.POSITION), 
        () -> m_manual.get(Button.LB).getAsBoolean(), 
        m_elevator));
    }
  }
  public double getManualElevatorPower() {
    return Functions.deadband(m_manual.get(Axis.LEFT_Y), OIConstants.kDeadband);
  }
}