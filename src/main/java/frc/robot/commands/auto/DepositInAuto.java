package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.Dunk;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.Node.NodeType;

public class DepositInAuto extends SequentialCommandGroup {

  /**
   * Deposits a game piece in the given position.
   */
  public DepositInAuto(Position depositPosition, NodeType type, Drivetrain drive, Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(drive, elevator, arm, intake);
    addCommands(
      new CalibrateElevator(elevator),
      new PositionIntake(elevator, arm, () -> (type == NodeType.CONE), depositPosition).withTimeout(1.5),
      type == NodeType.CONE
        ? new Dunk(arm, intake).withTimeout(1.5)
        : new Outtake(intake).withTimeout(1.5),
      new Stow(intake, elevator, arm)
    );
  }
    
}