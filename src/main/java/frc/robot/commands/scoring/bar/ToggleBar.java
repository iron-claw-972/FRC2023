package frc.robot.commands.scoring.bar;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Bar;
import frc.robot.subsystems.Bar.BarMode;
import frc.robot.subsystems.Bar.BarPosition;

public class ToggleBar extends SequentialCommandGroup {
  public ToggleBar(Bar bar) {
    addRequirements(bar);
    addCommands(
      new InstantCommand(() -> bar.setDesiredBarPosition(bar.getBarPosition() == BarPosition.DEPLOY ? BarPosition.STOW : BarPosition.DEPLOY), bar),
      new InstantCommand(() -> bar.setMode(BarMode.POSITION), bar)
    );
  }
}
