package frc.robot.commands.scoring.bar;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Bar;
import frc.robot.subsystems.Bar.BarMode;
import frc.robot.subsystems.Bar.BarPosition;

public class MoveBar extends SequentialCommandGroup {
  public MoveBar(Bar bar, BarPosition desiredBarPosition) {
    addRequirements(bar);
    addCommands(
      new InstantCommand(() -> bar.setDesiredBarPosition(desiredBarPosition), bar),
      new InstantCommand(() -> bar.setMode(BarMode.POSITION), bar)
    );
  }
}
