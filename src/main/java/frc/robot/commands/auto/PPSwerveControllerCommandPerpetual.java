package frc.robot.commands.auto;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class PPSwerveControllerCommandPerpetual extends FollowPathHolonomic {

    public PPSwerveControllerCommandPerpetual(
            PathPlannerPath path,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedSupplier,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            HolonomicPathFollowerConfig config,
            boolean useAllianceColor,
            Subsystem... requirements) {
        super(
                path,
                poseSupplier,
                speedSupplier,
                outputChassisSpeeds,
                config,
                ()->useAllianceColor,
                requirements
        );
    }

    public PPSwerveControllerCommandPerpetual(
        PathPlannerPath path,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedSupplier,
        Consumer<ChassisSpeeds> outputChassisSpeeds,
        PIDConstants translationConstatns,
        PIDConstants rotationConstants,
        double maxModuleSpeed,
        double driveBaseRadius,
        ReplanningConfig replanningConfig,
        boolean flipPath,
        Subsystem... requirements){
        super(
            path,
            poseSupplier,
            speedSupplier,
            outputChassisSpeeds,
            translationConstatns,
            rotationConstants,
            maxModuleSpeed,
            driveBaseRadius,
            replanningConfig,
            ()->flipPath,
            requirements
        );
    }

    public PPSwerveControllerCommandPerpetual(
        PathPlannerPath path,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speeSupplier,
        Consumer<ChassisSpeeds> outputChassisSpeeds,
        PIDConstants translationConstatns,
        PIDConstants rotationConstants,
        double maxModuleSpeed,
        double driveBaseRadius,
        double perioud,
        ReplanningConfig replanningConfig,
        boolean flipPath,
        Subsystem... requirements){
        super(
            path,
            poseSupplier,
            speeSupplier,
            outputChassisSpeeds,
            translationConstatns,
            rotationConstants,
            maxModuleSpeed,
            driveBaseRadius,
            perioud,
            replanningConfig,
            ()->flipPath,
            requirements
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}