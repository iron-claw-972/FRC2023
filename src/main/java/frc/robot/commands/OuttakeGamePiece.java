package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class OuttakeGamePiece extends CommandBase {

  private final Intake m_intake;
  
  public OuttakeGamePiece(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.intake(IntakeConstants.kOuttakeSpeed);
  }

  @Override
  public void end (boolean interrupted){
    m_intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return !m_intake.containsGamePiece();
  }
}
