package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.Dunk;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.AutoStartPosition;

/**
 * Deposits preloaded game piece, then exits community and engages with charge station.
 */
public class EngageNoPathplanner extends SequentialCommandGroup {

  public EngageNoPathplanner(SendableChooser<Position> depositSelector, SendableChooser<AutoStartPosition> startSelector, Drivetrain drive, Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(drive, elevator, arm, intake);

    Position depositPosition = depositSelector.getSelected();
    AutoStartPosition startPosition = startSelector.getSelected();
    
    addCommands(
      new CalibrateElevator(elevator),
      depositPosition == Position.MIDDLE ?
        new PositionIntake(elevator, arm, () -> false, depositPosition).withTimeout(1.5) : 
        new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).withTimeout(1).andThen(new PositionIntake(elevator, arm, () -> false, Position.TOP).withTimeout(1.5)),
      depositPosition == Position.MIDDLE ? 
        new Dunk(arm, intake).withTimeout(1.5) : new Outtake(intake).withTimeout(1.5),
      new Stow(intake, elevator, arm),
      startPosition == AutoStartPosition.LEFT
        ? new EngageFromLeftDriverSide(drive)
        : startPosition == AutoStartPosition.RIGHT
          ? new EngageFromRightDriverSide(drive)
          : new EngageFromCenterGrid(drive)
    );

  }

}
