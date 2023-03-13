package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

/**
 * Check some robot constants/parameters.
 */
public class ConstantsTest {

    @Test
    public void testRobotSize() {
        // The competition robot frame width and length is 26 inches.
        // It has 3/16 inch plates on all sides,
        // so frame width is 26.375!
        double widthFrame = Units.inchesToMeters(26.0);
        double widthFrameAndPlates = widthFrame + 2.0 * Units.inchesToMeters(3.0 / 16.0);

        // At Port Hueneme, the frame perimeter was 105 inches
        // frame perimeter is 105.5...
        assertEquals(105.0, 4 * Units.metersToInches(widthFrameAndPlates), 0.501);

        // Bumpers.
        // The backing board is 0.75 inches.
        // The noodles are 2.5 inches.
        // Board + noodles = 3.25 inches
        // Measure the red bumpers, and they are 3.5 inches.
        //
        // The latch studs are centered on the 1x2 frame rails. That's 0.5 inches
        // The bumper latch bracket holes for the studs are 3/4 in from the inside surface of the bumpers.
        // That puts the bumper inside surface at 3/4 - (0.5) = 1/4 inch out from widthFrame
        double thickBumpers = Units.inchesToMeters(3.5 + 0.25);

        // so width with bumpers is
        double widthFrameWithBumpers = widthFrame + 2 * thickBumpers;

        // check with values in DriveConstants
        // System.out.printf("widthFrameWithBumpers  = %8f %8f\n", widthFrameWithBumpers, Units.metersToInches(widthFrameWithBumpers));
        // System.out.printf("kRobotWidthWithBumpers = %8f %8f\n", DriveConstants.kRobotWidthWithBumpers, Units.metersToInches(DriveConstants.kRobotWidthWithBumpers));
        assertEquals(widthFrameWithBumpers, DriveConstants.kRobotWidthWithBumpers, 0.001);
    }

    /**
     * MK4i module
     * https://www.swervedrivespecialties.com/products/mk4i-swerve-module
     */
    enum SwerveDriveSpecialties {
        // Gearbox ratios from the SDS webpage
        L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
        L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
        L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0));

        /** Drive gear ratio varies for each module */
        final double driveRatio;
        /** Steering Gear ratio (same for all MK4i modules) */
        final double steerRatio = 150.0 / 7.0;

        private SwerveDriveSpecialties(double drive) {
            this.driveRatio = drive;
        }
    }

    @Test
    public void testSwerveRatios() {
        // check the mroe exact ratios against the published-to-2-digits ratios
        assertEquals(8.14, SwerveDriveSpecialties.L1.driveRatio, 0.01);
        assertEquals(6.75, SwerveDriveSpecialties.L2.driveRatio, 0.01);
        assertEquals(6.12, SwerveDriveSpecialties.L3.driveRatio, 0.01);

        // The drive ratio could be more accurate, but does not hurt
        assertEquals(SwerveDriveSpecialties.L2.driveRatio, DriveConstants.kDriveGearRatio, 0.01);

        // The steer ratio
        // print the relative error: 0.6e-4. After 100 rotations, error would be 0.6e-2 rotations (about 1.5 degrees)
        // System.out.println((DriveConstants.kSteerGearRatio - SwerveDriveSpecialties.L2.steerRatio) / SwerveDriveSpecialties.L2.steerRatio);
        assertEquals(SwerveDriveSpecialties.L2.steerRatio, DriveConstants.kSteerGearRatio, 0.01);
    }
}
