package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto: AutoFullRedTof", group ="Auto")
public class OmniAutoFullRedTof extends OmniAutoFullToF {
    @Override
    public void setFirstSkystoneValues(int position) {
        robot.stackFromSide = HardwareOmnibot.RobotSide.LEFT;
        baseAngle = 0.0;
        switch(position) {
            case 1:
                attackAngle = 40.0;
                sideDistance = 50.3;
                flyTime = 1300;
                flyBackTime = 1200;
                break;
            case 2:
                attackAngle = 40.0;
                sideDistance = 70.6;
                flyTime = 1400;
                break;
            case 3:
                attackAngle = 40.0;
                sideDistance = 91.0;
                flyTime = 1500;
                break;
        }
    }

    @Override
    public void setSecondSkystoneValues(int position) {
        // Stone 1 come from bridge side and go away.
        // Stone 2 and 3, come from the away and go to bridge.
        // This is due to range sensor limits.
        switch(position) {
            case 1:
                attackAngle = 45.0;
                sideDistance = 101.3;
                flyTime = 1300;
                break;
            case 2:
                attackAngle = -45.0;
                sideDistance = 51.3;
                flyTime = 1500;
                break;
            case 3:
                attackAngle = -45.0;
                sideDistance = 71.6;
                flyTime = 1600;
                break;
        }
    }
}
