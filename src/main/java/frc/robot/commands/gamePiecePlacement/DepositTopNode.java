package frc.robot.commands.gamePiecePlacement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.elevator.ExtendElevator;
import frc.robot.commands.intake.OuttakeGamePiece;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class DepositTopNode extends SequentialCommandGroup {
  public DepositTopNode(Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(elevator, arm, intake);
    addCommands(
      new ExtendElevator(elevator, ElevatorConstants.kTopNodeHeightExtension),
      new ExtendArm(arm, ArmConstants.kTopConeOuttakeAbsEncoderPos),
      new OuttakeGamePiece(intake), 
      new Stow(elevator, arm)
    );
  }
}
