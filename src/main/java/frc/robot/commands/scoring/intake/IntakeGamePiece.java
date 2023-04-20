package frc.robot.commands.scoring.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.util.Blinkin;
import frc.robot.util.GamePieceType;
import frc.robot.util.Blinkin.Colors;

public class IntakeGamePiece extends CommandBase {

  private final Intake m_intake; 
  private GamePieceType m_type;
  private final BooleanSupplier m_isCone;
  private final boolean m_runsForever;
  private Debouncer m_stallDebouncer = new Debouncer(IntakeConstants.kIntakeStallTime, DebounceType.kBoth);

  /**
   * Spins the intake until the game piece is inside the intake.
   * @param intake the intake subsystem
   * @param gamePieceType intaking a cone or cube?
   * @param runsForever if the command should end, or run forever. If false, will end based on motor currents
   */
  public IntakeGamePiece(Intake intake, GamePieceType gamePieceType, boolean runsForever) {
    this(intake, () -> gamePieceType == GamePieceType.CONE, runsForever);
  }

  /**
   * Spins the intake until the game piece is inside the intake.
   * @param intake the intake subsystem
   * @param isCone a supplier that when the command starts, checks if will intake a cone or cube
   * @param runsForever if the command should end, or run forever. If false, will end based on motor currents
   */
  public IntakeGamePiece(Intake intake, BooleanSupplier isCone, boolean runsForever) {
    m_intake = intake;
    m_isCone = isCone;
    m_runsForever = runsForever;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_type = m_isCone.getAsBoolean() ? GamePieceType.CONE : GamePieceType.CUBE;
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
    if (RobotBase.isSimulation()) return true;

    boolean currentAtStopPoint = true;

    if (m_type == GamePieceType.CUBE) {
      currentAtStopPoint = m_stallDebouncer.calculate(
        m_intake.getCurrent() >= IntakeConstants.kCubeIntakeCurrentStopPoint
      );
    } else if (m_type == GamePieceType.CONE) {
      currentAtStopPoint = m_stallDebouncer.calculate(
        m_intake.getCurrent() >= IntakeConstants.kConeIntakeCurrentStopPoint
      );
    }

    if (currentAtStopPoint) {
      if (m_type == GamePieceType.CONE) {
        Blinkin.blinkColor(Colors.YELLOW);
      } else {
        Blinkin.blinkColor(Colors.VIOLET);
      }
    } else {
      if (m_type == GamePieceType.CONE) {
        Blinkin.setColor(Colors.YELLOW);
      } else {
        Blinkin.setColor(Colors.VIOLET);
      }
    }

    if (m_runsForever) return false;
    return currentAtStopPoint;
  }

}
