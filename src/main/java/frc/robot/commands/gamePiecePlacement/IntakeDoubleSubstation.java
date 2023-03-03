package frc.robot.commands.gamePiecePlacement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.elevator.ExtendElevator;
import frc.robot.commands.intake.IntakeGamePiece;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class IntakeDoubleSubstation extends SequentialCommandGroup {
  //TODO: add javadoc comment
  public IntakeDoubleSubstation(Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(elevator, arm, intake);
    addCommands(
      new ExtendElevator(elevator, ElevatorConstants.kDoubleSubstationHeightExtension),
      new ExtendArm(arm, ArmConstants.kShelfPositionAbsEncoderPos),
      new IntakeGamePiece(intake), 
      new Stow(elevator, arm)
      //move the drivetrain back after this command group ends
    );
  }
}
