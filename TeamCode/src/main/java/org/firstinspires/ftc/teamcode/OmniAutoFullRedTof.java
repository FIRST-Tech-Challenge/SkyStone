package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto: AutoFullRedTof", group ="Auto")
public class OmniAutoFullRedTof extends OmniAutoFullToF {
    @Override
    public void setSkystoneValues(int position) {
        robot.stackFromSide = HardwareOmnibot.RobotSide.LEFT;
        baseAngle = 0.0;
        switch(position) {
            case 1:
			    // For first Skystone
                attackAngle1 = 38.0;
                sideDistance1 = 55.3;
                flyTime1 = 1300;
                flyBackTime1 = 1200;
				// For second Skystone
				attackAngle2 = 40.0;
				sideDistance2 = 101.3;
				flyTime2 = 1300;
                break;
            case 2:
			    // For first Skystone
                attackAngle1 = 38.0;
                sideDistance1 = 75.6;
                flyTime1 = 1400;
                flyBackTime1 = 1200;
				// For second Skystone
				attackAngle2 = -40.0;
				sideDistance2 = 51.3;
				flyTime2 = 1500;
                break;
            case 3:
			    // For first Skystone
                attackAngle1 = 38.0;
                sideDistance1 = 96.0;
                flyTime1 = 1500;
                flyBackTime1 = 1200;
				// For second Skystone
				attackAngle2 = -40.0;
				sideDistance2 = 71.6;
				flyTime2 = 1400;
                break;
        }
    }
}
