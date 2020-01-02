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
        fudgeAngle = 4;
        baseAngle = 180.0;
        secondSkystone = false;
        switch(position) {
            case 1:
                // For first Skystone
                attackAngle1 = 140.0;
                sideDistance1 = 52.3;
                flyTime1 = 1300;
                flyBackTime1 = 1900;
                // For second Skystone
                attackAngle2 = 220.0;
                sideDistance2 = 40.3;
                flyTime2 = 1100;
                flyBackTime2 = 750;
                break;
            case 2:
                // For first Skystone
                attackAngle1 = 140.0;
                sideDistance1 = 75.6;
                flyTime1 = 1100;
                flyBackTime1 = 1900;
                // For second Skystone
                attackAngle2 = 220.0;
                sideDistance2 = 40.3;
                flyTime2 = 1100;
                flyBackTime2 = 750;
                break;
            case 3:
                // For first Skystone
                attackAngle1 = 140.0;
                sideDistance1 = 91.9;
                flyTime1 = 800;
                flyBackTime1 = 1900;
                // For second Skystone
                attackAngle2 = 220.0;
                sideDistance2 = 40.3;
                flyTime2 = 1100;
                flyBackTime2 = 750;
                break;
        }
    }
}
