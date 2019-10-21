package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.robots.Robot;

public class RuckusUtils {
    public static int getCubePostition(int goldMineralX, int silverMineral1X, int silverMineral2X){
        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                return Robot.LEFT;

            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                return Robot.RIGHT;

            } else {
                return Robot.CENTRE;
            }
        }

        else{
            return -1;
        }
    }

}
