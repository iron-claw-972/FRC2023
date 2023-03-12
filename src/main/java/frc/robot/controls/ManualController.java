package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorMode;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.RollerIntake.IntakePiece;
import frc.robot.subsystems.RollerIntake.IntakeMode;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class ManualController {
  GameController m_manual = new GameController(OIConstants.kManualJoy);
  private FourBarArm m_arm;
  private RollerIntake m_intake;
  private Elevator m_elevator;
  
  public ManualController(FourBarArm arm, RollerIntake intake, Elevator elevator) {
    m_arm = arm;
    m_intake = intake;
    m_elevator = elevator;
  }
  
  public void configureControls() {
    if (m_intake != null) {
      m_manual.get(Button.A).onTrue(new InstantCommand(()-> m_intake.setIntakeMode(IntakeMode.INTAKE_CUBE))).onFalse(new InstantCommand(()-> m_intake.setIntakeMode(IntakeMode.DISABLED)));
      m_manual.get(Button.B).onTrue(new InstantCommand(()-> m_intake.setIntakeMode(IntakeMode.OUTTAKE_CUBE))).onFalse(new InstantCommand(()-> m_intake.setIntakeMode(IntakeMode.DISABLED)));
      m_manual.get(Button.X).onTrue(new InstantCommand(()-> m_intake.setIntakeMode(IntakeMode.INTAKE_CONE))).onFalse(new InstantCommand(()-> m_intake.setIntakeMode(IntakeMode.DISABLED)));
      m_manual.get(Button.Y).onTrue(new InstantCommand(()-> m_intake.setIntakeMode(IntakeMode.OUTTAKE_CONE))).onFalse(new InstantCommand(()-> m_intake.setIntakeMode(IntakeMode.DISABLED)));
  
      
      // Commented out due to outdated code
      // TODO: Update to new intake commands
      // m_manual.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakePower), m_intake));
      // m_manual.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakePower),m_intake));
      // m_manual.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stopIntake(), m_intake));  
    }

    if (m_arm != null) {
      m_manual.get(Button.Y).onTrue(new InstantCommand(() -> m_arm.setMotorPower(0.05)));
      m_manual.get(Button.X).onTrue(new InstantCommand(() -> m_arm.setMotorPower(-0.05)));
      m_manual.get(Button.B).onTrue(new InstantCommand(() -> m_arm.setMotorPower(0)));  
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