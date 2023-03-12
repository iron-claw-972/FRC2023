package frc.robot.commands.scoring;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Wrist;

public class WristDunk extends SequentialCommandGroup {
    /**
     * Moves the wrist down and releases the intake, to allow dunking the cone on the node
     */
    public WristDunk(Wrist wrist, RollerIntake intake) {
        addRequirements(wrist, intake);
        addCommands(
            // change intake type if needed
            new RotateWrist(wrist, WristConstants.kDunkPos),
            new InstantCommand(() -> intake.getIntakeMotor().setNeutralMode(NeutralMode.Coast))
        );
    }
}
