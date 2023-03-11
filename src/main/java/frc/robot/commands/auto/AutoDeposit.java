package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class AutoDeposit extends SequentialCommandGroup {
  public AutoDeposit(Position position, Drivetrain drive, Elevator elevator, FourBarArm arm, Intake intake, boolean isCone) {
    addRequirements(drive, elevator, arm, intake);
    addCommands(
      new MoveElevator(elevator, 
        position == Position.BOTTOM  && isCone ? ElevatorConstants.kBottomConeHeight : 
        position == Position.MIDDLE && isCone ? ElevatorConstants.kMiddleConeHeight :
        position == Position.TOP && isCone ? ElevatorConstants.kTopConeHeight : 
        position == Position.BOTTOM && !isCone ? ElevatorConstants.kBottomCubeHeight :
        position == Position.MIDDLE && !isCone ? ElevatorConstants.kMiddleCubeHeight :
       ElevatorConstants.kTopCubeHeight 
      ),
      new PositionIntake(elevator, arm, () -> isCone, position).withTimeout(1),
      new Outtake(intake), 
      new Stow(intake, elevator, arm) 
    );
  }

}
  
  


