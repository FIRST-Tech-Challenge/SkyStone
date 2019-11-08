package org.firstinspires.ftc.teamcode;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class MecanumMoveTest {
    public static final double MAX_DBL_DELTA = 0.001;
    @Test
    public void testMecanumError() {
        RobotNavigator navigator = new RobotNavigator(null);
        PIDMecanumMoveTask task = new PIDMecanumMoveTask(null, null, null);
        task.setPath(new RobotPosition(-1, -1, 0), new RobotPosition(-11, -11, 0));
        navigator.setWorldPosition(-5, -5);
        System.out.println(task.getPosError());

        task.prepare();

        assertEquals (0, task.getPosError(), MAX_DBL_DELTA);

        navigator.setWorldPosition(-5, -6);
        assertEquals (-Math.sqrt(2)/2, task.getPosError(), MAX_DBL_DELTA);

        task.setPath(new RobotPosition(0, 0, 0), new RobotPosition(10, 10*Math.sqrt(3), 0));

        task.prepare();

        navigator.setWorldPosition(5, 5*Math.sqrt(3));
        assertEquals (0, task.getPosError(), MAX_DBL_DELTA);

        navigator.setWorldPosition(6, 5*Math.sqrt(3));
        assertEquals (Math.sqrt(3)/2, task.getPosError(), MAX_DBL_DELTA);
    }
}