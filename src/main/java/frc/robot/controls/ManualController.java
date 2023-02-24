package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class ManualController {
  
  GameController m_manual = new GameController(OIConstants.kManualJoy);
  private FourBarArm m_arm;
  private Intake m_intake;
  private Drivetrain m_drive;
  
  public ManualController(FourBarArm arm, Intake intake, Drivetrain drive){
    m_arm = arm;
    m_intake = intake;
    m_drive = drive;
  }
  
  public void configureControls() {
    m_manual.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed), m_intake));
    m_manual.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakeSpeed),m_intake));
    m_manual.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stop(), m_intake));
    
    m_manual.get(Button.Y).onTrue(new InstantCommand(() -> m_arm.setMotorPower(0.05)));
    m_manual.get(Button.X).onTrue(new InstantCommand(() -> m_arm.setMotorPower(-0.05)));
    m_manual.get(Button.B).onTrue(new InstantCommand(() -> m_arm.setMotorPower(0)));

    m_manual.get(GameController.Button.RB).onTrue(new BalanceCommand(m_drive));
  }
}