package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.constants.swerve.ModuleType;

public class ModuleTypeTest {

    /** Make sure that ModuleType.id returns the same as ModuleType.getID() */
    @Test
    public void testEnums() {
        for (ModuleType mt : ModuleType.values()) {
            // @SuppressWarnings("deprecated")
            assertEquals(mt.id, mt.getID());
        }
    }

    /** Make sure that ModuleType.abbrev returns the same as ModuleType.getAbbreviation() */
    @Test
    public void testAbbreviations() {
        for (ModuleType mt : ModuleType.values()) {
            // @SuppressWarnings("deprecated")
            assertEquals(mt.abbrev, mt.getAbbreviation());
        }

    }
}
