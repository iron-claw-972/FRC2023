package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.AutoStartPosition;
import frc.robot.util.Node.NodeType;

public class AroundChargeStation extends SequentialCommandGroup {

  private void run(Position depositPosition, NodeType type, AutoStartPosition startPosition, Drivetrain drive, Elevator elevator, FourBarArm arm, Intake intake) {
    addCommands(
      new DepositInAuto(depositPosition, type, drive, elevator, arm, intake),
      startPosition == AutoStartPosition.LEFT
        ? new PathPlannerCommand("Around the Charge Station (Top)", 0, drive).withTimeout(2)
        : new PathPlannerCommand("Around the Charge Station (Bottom)", 0, drive).withTimeout(2),
      new IntakeGamePiece(intake).withTimeout(1.5),
      
    );
  }

  public AroundChargeStation(Position depositPosition,  NodeType type, Drivetrain drive, Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(drive, elevator, arm, intake);

    addCommands(
    );
  }

  public AroundChargeStation(SendableChooser<Position> depositSelector, SendableChooser<NodeType> typeSelector, Drivetrain drive, Elevator elevator, FourBarArm arm, Intake intake) {
    this(depositSelector.getSelected(), typeSelector.getSelected(), drive, elevator, arm, intake);
  }
    
}
