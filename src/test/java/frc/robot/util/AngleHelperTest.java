package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.Pair;
import java.util.ArrayList;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AngleHelperTest {
    private TargetAngle m_targetAngle;

    @BeforeEach
    void setup() {
        ArrayList<Pair<Double, Double>> angles = new ArrayList<>();
        // Populate the table with example values
        angles.add(new Pair<>(1.0, 10.0));
        angles.add(new Pair<>(2.0, 20.0));
        angles.add(new Pair<>(3.0, 30.0));
        angles.add(new Pair<>(4.0, 40.0));

        m_targetAngle = new TargetAngle(angles);
    }

    @Test
    void testInterpolationWithinTable() {
        double result = m_targetAngle.getAngleHelper(2.5);
        assertEquals(25.0, result, "Angle should be correctly interpolated within table.");
    }

    @Test
    void testBeforeFirstEntry() {
        double result = m_targetAngle.getAngleHelper(0.5);
        assertEquals(5.0, result, "Angle should be interpolated before the first table entry.");
    }

    @Test
    void testAfterLastEntry() {
        double result = m_targetAngle.getAngleHelper(4.5);
        assertEquals(45.0, result, "Angle should be interpolated after the last table entry.");
    }

    @Test
    void testExactMatchFirstEntry() {
        double result = m_targetAngle.getAngleHelper(1.0);
        assertEquals(10.0, result, "Angle should match the first entry in the table.");
    }

    @Test
    void testExactMatchLastEntry() {
        double result = m_targetAngle.getAngleHelper(4.0);
        assertEquals(40.0, result, "Angle should match the last entry in the table.");
    }

    @Test
    void testEmptyTable() {
        m_targetAngle = new TargetAngle(new ArrayList<>());
        double result = m_targetAngle.getAngleHelper(2.5);
        assertEquals(0, result, "Angle should return default value with empty table.");
    }

    @Test
    void testZeroDistance() {
        double result = m_targetAngle.getAngleHelper(0.0);
        assertEquals(0, result, "Angle should return default value for zero distance.");
    }

    @Test
    void testNegativeDistance() {
        double result = m_targetAngle.getAngleHelper(-1.0);
        assertEquals(0, result, "Angle should return default value for negative distance.");
    }

    @Test
    void testDistanceOf1Point1() {
        double result = m_targetAngle.getAngleHelper(1.1);
        assertEquals(11.0, result, "Angle should be correctly interpolated within table.");
    }
}
