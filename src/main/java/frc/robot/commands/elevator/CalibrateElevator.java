package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;

public class CalibrateElevator extends SequentialCommandGroup {
  /**
   * Calibrate the elevator by resetting the encoder after it hits the bottom
   * limit switch. 
   * @param elevator
   */
  public CalibrateElevator(Elevator elevator) {
    addCommands(
      new ResetEncoderAtBottom(elevator), 
      new GetMaxHeight(elevator),
      new InstantCommand(()->elevator.setEnabled(true)),
      new InstantCommand(() -> elevator.setIsCalibrated())
    );
  }

  
    






}
