import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.RobotContainer;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotContainerTest {

    RobotContainer frcRobot;

    @BeforeEach
    public void setup() {
        HAL.initialize(500, 0);
    }

    // TODO: unsatisfiedlinkerror orignates @ line 21
    @Test
    public void test() {
        frcRobot = new RobotContainer();
        assertEquals(1, 1);
    }
}