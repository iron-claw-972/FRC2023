package frc.robot.commands.gamePiecePlacement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.intake.IntakeGamePiece;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class IntakeFromGround extends SequentialCommandGroup {
  //TODO: add javadoc comment
  public IntakeFromGround(Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(elevator, arm, intake);
    addCommands(
      new MoveElevator(elevator, ElevatorConstants.kGroundIntakeExtension),
      new ExtendArm(arm, ArmConstants.kBottomNodePositionAbsEncoderPos),
      new IntakeGamePiece(intake),
      new Stow(elevator, arm)
    );
  }
}
