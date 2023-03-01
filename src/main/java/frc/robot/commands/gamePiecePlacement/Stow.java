package frc.robot.commands.gamePiecePlacement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.elevator.ExtendElevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;

public class Stow extends SequentialCommandGroup {
  //TODO: add javadoc comment
  public Stow(Elevator elevator, FourBarArm arm) {
    addRequirements(elevator, arm);
    addCommands(
      new ExtendArm(arm, ArmConstants.kStowedAbsEncoderPos),
      new ExtendElevator(elevator, ElevatorConstants.kMinExtension)
    );
  }
}
