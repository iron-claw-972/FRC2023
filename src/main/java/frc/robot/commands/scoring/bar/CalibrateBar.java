package frc.robot.commands.scoring.bar;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Bar;
import frc.robot.subsystems.Bar.BarMode;
import frc.robot.constants.BarConstants;

public class CalibrateBar extends SequentialCommandGroup {
  public CalibrateBar(Bar bar) {
    addRequirements(bar);
    addCommands(
      new InstantCommand(() -> bar.zeroEncoderAtStow()),
      new InstantCommand(() -> bar.setIsCalibrated()),
      new InstantCommand(() -> bar.setIdleMode(BarConstants.kMotorIdleMode)),
      new InstantCommand(() -> bar.setMode(BarMode.DISABLED))
    );
  }
}
