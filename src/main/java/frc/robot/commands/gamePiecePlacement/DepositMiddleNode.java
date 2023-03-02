package frc.robot.commands.gamePiecePlacement;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.intake.OuttakeGamePiece;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class DepositMiddleNode extends SequentialCommandGroup {
  //TODO: add javadoc comment
  public DepositMiddleNode(Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(elevator, arm, intake);
    addCommands(
      new MoveElevator(elevator, ElevatorConstants.kMiddleNodeHeightExtension),
      new ExtendArm(arm, ArmConstants.kMiddleConeOuttakeAbsEncoderPos),
      new OuttakeGamePiece(intake), 
      new Stow(elevator, arm)
    );
  }
}
