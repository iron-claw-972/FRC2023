package frc.robot.commands.scoring;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.arm.ExtendArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class Stow extends ParallelCommandGroup {
  public Stow(Intake intake, Elevator elevator, FourBarArm arm) {
    addRequirements(intake, elevator, arm);
    addCommands(
      new InstantCommand(() -> intake.setIdleMode(IdleMode.kBrake)),
      new ExtendArm(arm, ArmConstants.kStowPos),
      new PositionIntake(elevator, arm, intake::hasCone, Position.STOW)
    );
  }
}