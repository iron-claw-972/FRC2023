package frc.robot.commands.CommandGroups;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.elevator.MoveToExtension;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class DepositMiddleNode extends SequentialCommandGroup {
  public DepositMiddleNode(Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(elevator, arm, intake);
    addCommands(
      new SequentialCommandGroup(
        new MoveToExtension(elevator, ElevatorConstants.kMiddleNodeHeightExtension),
        new ExtendToPosition(arm, ArmConstants.kMiddlePosition),
        //TODO: Intake command here
        new Stow(elevator, arm)
      )
    );
  }
}
