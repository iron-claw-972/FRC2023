package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

/**
 * Represents a game piece that is staged on the field before the match starts.
 */
public class StagedGamePiece {

  public final Alliance m_alliance;
  public final int m_id;
  public final Translation2d m_translation;
  public final GamePieceType m_type;

  /**
   * Represents a new staged game piece.
   * The index determines which staged game piece to create. Id 1 is the game piece closest to the scoring table, and id 4 is the game piece furthest from the scoring table.
   * 
   * @param alliance the alliance color 
   * @param id the index of the piece
   * @param type the type of game piece
   * @throws IllegalArgumentException if id is not between 1 and 4 inclusive
   */
  public StagedGamePiece(Alliance alliance, int id, GamePieceType type) {

    if (type == GamePieceType.NONE) {
      throw new IllegalArgumentException("Game piece type cannot be NONE.");
    }

    if (id < 1 || id > 4) {
      throw new IllegalArgumentException("The id must be between 1 and 4 inclusive.");
    }

    m_alliance = alliance;
    m_id = id;
    m_type = type;

    double x = FieldConstants.kFieldWidth / 2;
    if (m_alliance == Alliance.Blue) {
      x -= FieldConstants.kCenterToStagedPieceX;
    } else {
      x += FieldConstants.kCenterToStagedPieceX;
    }

    // Values here obtained from game manual.
    double y = Units.feetToMeters(3.25 + ((m_id - 1) * 4));

    m_translation = new Translation2d(x, y);

  }
  
}
