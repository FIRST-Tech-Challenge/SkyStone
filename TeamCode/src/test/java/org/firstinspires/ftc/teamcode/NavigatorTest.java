package org.firstinspires.ftc.teamcode;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class NavigatorTest {
    public static final double MAX_DBL_DELTA = 0.001;
    @Test
    public void testNavigator () {
        RobotProfile profile = new RobotProfile();
        profile.hardwareSpec = profile.new HardwareSpec();
        profile.hardwareSpec.trackWheelDiameter = 100;
        profile.hardwareSpec.leftRightWheelDist = 300;
        RobotNavigator robotNavigator = new RobotNavigator(profile);

        robotNavigator.setInitPosition(0,0,0);
        robotNavigator.setInitEncoderCount(0,0,0);
        robotNavigator.updateAllPositions(61,52, 2);
        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY() +
                " , angle: " + Math.toDegrees(robotNavigator.getHeading()));

        robotNavigator.updateAllPositions(100,100,0);
        assertEquals (0, robotNavigator.getWorldX(), MAX_DBL_DELTA);
        assertEquals (100, robotNavigator.getWorldY(), MAX_DBL_DELTA);
        assertEquals (0, robotNavigator.getHeading(), MAX_DBL_DELTA);

        robotNavigator.setInitPosition(0,0,Math.PI/4);
        robotNavigator.setInitEncoderCount(0,0,0);
        robotNavigator.updateAllPositions(100,100,0);
        assertEquals (100/Math.sqrt(2),robotNavigator.getWorldX(), MAX_DBL_DELTA);
        assertEquals (100/Math.sqrt(2), robotNavigator.getWorldY(), MAX_DBL_DELTA);
        assertEquals (Math.PI/4, robotNavigator.getHeading(), MAX_DBL_DELTA);

        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY());
        robotNavigator.updateAllPositions(30,15,0);
        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY());
        robotNavigator.updateAllPositions(45,45,0);
        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY());
        robotNavigator.updateAllPositions(45,45,10);
        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY());
        robotNavigator.updateAllPositions(60,70,20);
        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY());
        robotNavigator.updateAllPositions(75,80,30);
        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY() +
                " , angle: " + Math.toDegrees(robotNavigator.getHeading()));
        robotNavigator.updateAllPositions(80,75,30);
        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY() +
                " , angle: " + Math.toDegrees(robotNavigator.getHeading()));
        robotNavigator.updateAllPositions(75,80,30);
        System.out.println("worldX: " + robotNavigator.getWorldX() + ", worldY: " + robotNavigator.getWorldY() +
                " , angle: " + Math.toDegrees(robotNavigator.getHeading()));


        robotNavigator.setInitPosition(0,0,0);
        robotNavigator.setInitEncoderCount(0,0,0);
        robotNavigator.updateAllPositions(30,15,0);
//        assertEquals (10/Math.sqrt(2),robotNavigator.getWorldX(), MAX_DBL_DELTA);
//        assertEquals (10/Math.sqrt(2), robotNavigator.getWorldY(), MAX_DBL_DELTA);
//        assertEquals (45, robotNavigator.getHeading(), MAX_DBL_DELTA);

    }
}