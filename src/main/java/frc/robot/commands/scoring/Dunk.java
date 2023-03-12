package frc.robot.commands.scoring;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.arm.ExtendArm;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class Dunk extends SequentialCommandGroup {
    /*
     * Moves the arm down and releases the intake, to allow dunking the cone on the node
     */
    public Dunk(FourBarArm arm, Intake intake) {
        addRequirements(arm, intake);
        addCommands(
            new ExtendArm(arm, ArmConstants.kDunkPos),
            new InstantCommand(() -> intake.setNeutralMode(NeutralMode.Coast))
        );
    }
}
