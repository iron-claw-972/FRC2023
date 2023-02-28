package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;

public class CalibrateElevator extends SequentialCommandGroup {
  
  /**
   * Sequential command group that runs the commands {@link ResetEncoderAtBottom} and {@link CalibrateMaxExtension} to calibrate the elevator.
   */
  public CalibrateElevator(Elevator elevator) {
    addCommands(
      new ResetEncoderAtBottom(elevator), 
      new CalibrateMaxExtension(elevator),
      new InstantCommand(() -> elevator.setPIDEnabled(true)),
      new InstantCommand(() -> elevator.setIsCalibrated())
    );
  }
}
