package frc.robot.commands.scoring.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.RollerIntake.IntakeMode;
import frc.robot.util.GamePieceType;

public class IntakeGamePiece extends CommandBase {

  private final RollerIntake m_intake; 
  private GamePieceType m_type;
  private final BooleanSupplier m_isCone;
  private final boolean m_runsForever;
  private Debouncer m_stallDebouncer = new Debouncer(IntakeConstants.kIntakeStallTime, DebounceType.kBoth);

  /**
   * Spins the intake until the game piece is inside the intake.
   * @param intake the intake subsystem
   * @param runsForever if the command should end, or run forever. If false, will end based on motor currents
   * @param isCone a supplier that when the command starts, checks if will intake a cone or cube
   */
  public IntakeGamePiece(RollerIntake intake, boolean runsForever, BooleanSupplier isCone) {
    m_intake = intake;
    m_isCone = isCone;
    m_runsForever = runsForever;
    addRequirements(m_intake);
  }

  /**
   * Spins the intake until the game piece is inside the intake.
   * @param intake the intake subsystem
   * @param runsForever if the command should end, or run forever. If false, will end based on motor currents
   * @param type the type of game piece to intake
   */
  public IntakeGamePiece(RollerIntake intake, boolean runsForever, GamePieceType type) {
    m_intake = intake; 
    m_type = type;
    m_runsForever = runsForever;
    m_isCone = m_type == GamePieceType.CONE ? () -> true : () -> false;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    if (m_type == null) {
      m_type = m_isCone.getAsBoolean() ? GamePieceType.CONE : GamePieceType.CUBE;
    }
    if (m_type == GamePieceType.CUBE) {
      m_intake.setMode(IntakeMode.INTAKE_CUBE);
    } else if (m_type == GamePieceType.CONE) {
      m_intake.setMode(IntakeMode.INTAKE_CONE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setHeldGamePiece(m_type);
    m_intake.setMode(IntakeMode.DISABLED);
  }
  
  @Override
  public boolean isFinished() {
    if (m_runsForever) return false;

    if (m_type == GamePieceType.CUBE) {
      return m_stallDebouncer.calculate(
        m_intake.getCurrent() >= IntakeConstants.kCubeIntakeCurrentStopPoint
      );
    } else if (m_type == GamePieceType.CONE) {
      return m_stallDebouncer.calculate(
        m_intake.getCurrent() >= IntakeConstants.kConeIntakeCurrentStopPoint
      );
    }
    DriverStation.reportWarning("IntakeGamePiece Command missing GamePieceType", false);
    return true;
  }

}
