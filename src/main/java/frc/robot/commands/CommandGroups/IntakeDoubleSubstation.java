package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.elevator.MoveToExtension;
import frc.robot.commands.intake.IntakeGamePiece;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class IntakeDoubleSubstation extends SequentialCommandGroup {
  public IntakeDoubleSubstation(Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(elevator, arm, intake);
    addCommands(
      new MoveToExtension(elevator, ElevatorConstants.kDoubleSubstationHeightExtension),
      new ExtendToPosition(arm, ArmConstants.kShelfPosition),
      new IntakeGamePiece(intake), 
      new ExtendToPosition(arm, ArmConstants.kInitialPosition)
      //move the drivetrain back after this command group ends
    );
  }
}
