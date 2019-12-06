package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name="Auto: AutoFullBlueTof", group ="Auto")
public class OmniAutoFullBlueTof extends OmniAutoFullToF {
    @Override
    public void setVisionPoints() {
        sub2PointA = new Point(185, 23); // -25 Stone2, Position 2
        sub2PointB = new Point(195, 33);
        sub3PointA = new Point(185, 99); // -50 Stone3, Position 3
        sub3PointB = new Point(195, 109);
        sub1PointA = new Point(185, 164); //-106 Stone4, Position 1
        sub1PointB = new Point(195, 174);
    }

    @Override
    public void setSkystoneValues(int position) {
        robot.stackFromSide = HardwareOmnibot.RobotSide.LEFT;
        baseAngle = 180.0;
        switch(position) {
            case 1:
			    // For first Skystone
                attackAngle1 = 38.0;
                sideDistance1 = 55.3;
                flyTime1 = 1500;
                flyBackTime1 = 1200;
				// For second Skystone
				attackAngle2 = 38.0;
				sideDistance2 = 101.3;
				flyTime2 = 1300;
                break;
            case 2:
			    // For first Skystone
                attackAngle1 = 38.0;
                sideDistance1 = 75.6;
                flyTime1 = 1400;
                flyBackTime1 = 1550;
				// For second Skystone
				attackAngle2 = -40.0;
				sideDistance2 = 47.3;
				flyTime2 = 1200;
                break;
            case 3:
			    // For first Skystone
                attackAngle1 = 38.0;
                sideDistance1 = 96.0;
                flyTime1 = 1300;
                flyBackTime1 = 1200;
				// For second Skystone
				attackAngle2 = -38.0;
				sideDistance2 = 65.6;
				flyTime2 = 1400;
                break;
        }
    }
}
