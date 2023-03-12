package frc.robot.commands.scoring;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class WristStow extends SequentialCommandGroup {
    /*
     * stows the elevator, wrist, and intake
     */
    public WristStow(Intake intake, Wrist wrist, Elevator elevator) {
        addRequirements(intake, wrist, elevator);
        addCommands(
            // change intake type if needed
            new InstantCommand(() -> intake.setIdleMode(IdleMode.kBrake)),
            new RotateWrist(wrist, WristConstants.kStowPos),
            new MoveElevator(elevator, ElevatorConstants.kStowHeight)
        );
    }
}
