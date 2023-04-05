package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class Stow extends SequentialCommandGroup {
  /**
   * Puts the elevator and wrist into the stow position.
   * @param elevator the elevator subsystem
   * @param wrist the wrist subsystem
   */
  public Stow(Elevator elevator, Wrist wrist) {
    addRequirements(elevator, wrist);
    // isCone doesn't matter for Stow
    addCommands(new PositionIntake(elevator, wrist, () -> false, Position.STOW));
  }
}
