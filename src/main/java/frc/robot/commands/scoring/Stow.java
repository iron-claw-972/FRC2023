package frc.robot.commands.scoring;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.arm.ExtendArm;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.RollerIntake;

public class Stow extends SequentialCommandGroup {
  public Stow(RollerIntake intake, Elevator elevator, FourBarArm arm) {
    addRequirements(intake, elevator, arm);
    addCommands(
      new ExtendArm(arm, ArmConstants.kStowPos),
      new MoveElevator(elevator, ElevatorConstants.kStowHeight)
    );
  }
}