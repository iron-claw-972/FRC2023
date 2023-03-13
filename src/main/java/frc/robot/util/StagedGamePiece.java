package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

/**
 * Represents a game piece that is staged on the field before the match starts. The {@link Pose2d} for the game piece is relative to the field origin.
 * The Pose2d will contain the Translation2d of the game piece and the Rotation2d of the game piece. The rotation will be the angle from the center line to the respective alliance color's grid.
 */
public class StagedGamePiece {

  public final Alliance m_alliance;
  public final int m_id;
  public final Pose2d m_pose;

  /**
   * Represents a new staged game piece.
   * The index determines which staged game piece to create. Id 1 is the game piece closest to the scoring table, and id 4 is the game piece furthest from the scoring table.
   * 
   * @param alliance the alliance color 
   * @param id the index of the piece
   * @throws IllegalArgumentException if id is not between 1 and 4 inclusive
   */
  public StagedGamePiece(Alliance alliance, int id) {

    if (id < 1 || id > 4) {
      throw new IllegalArgumentException("The id must be between 1 and 4 inclusive.");
    }

    m_alliance = alliance;
    m_id = id;

    double x = FieldConstants.kFieldWidth / 2;
    if (m_alliance == Alliance.Blue) {
      x -= FieldConstants.kCenterToStagedPieceX;
    } else {
      x += FieldConstants.kCenterToStagedPieceX;
    }

    // Values here obtained from game manual.
    double y = Units.feetToMeters(3.25 + ((m_id - 1) * 4));

    Rotation2d rotation = new Rotation2d();
    if (m_alliance == Alliance.Blue) {
      rotation = rotation.plus(Rotation2d.fromDegrees(180));
    }

    m_pose = new Pose2d(new Translation2d(x, y), rotation);


  }
  
}
