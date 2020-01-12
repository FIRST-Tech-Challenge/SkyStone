/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;
import org.opencv.core.Point;
import org.firstinspires.ftc.teamcode.HelperClasses.CurvePoint;

/**
 * Created by 12090 STEM Punk
 */
@Autonomous(name="Auto: Blue Full Auto", group ="Auto")
public class OmniAutoFullXYBlue extends OmniAutoFullXYOdo
{
    // Sets the points in the image to detect the skystone.
    @Override
    public void setVisionPoints() {
        sub2PointA = new Point(174, 23); // -25 Stone2, Position 2
        sub2PointB = new Point(184, 33);
        sub3PointA = new Point(174, 99); // -50 Stone3, Position 3
        sub3PointB = new Point(184, 109);
        sub1PointA = new Point(174, 164); //-106 Stone4, Position 1
        sub1PointB = new Point(184, 174);
    }

    protected double skystoneX = 89.22872;
    protected double skystone1Y = 63.66;
    protected double skystone2Y = skystone1Y + 20.32;
    protected double skystone3Y = skystone2Y + 20.32;
    protected double skystone4Y = skystone3Y + 20.32;
    protected double skystone5Y = skystone4Y + 20.32;
    protected double skystone6Y = skystone5Y + 20.32;
    protected double runLaneX = 92.19186;

    protected double attackAngle = Math.toRadians(-45.0);
    protected double runAngle = Math.toRadians(-90.0);

    // Sets all the route points for executing the autonomous.
    @Override
    public void setSkystoneValues(int position) {
        // Robot starting location
        startLocation = new WayPoint(22.86, 83.14436, Math.toRadians(0.0), 0.0, false);

        // small pull away from wall to rotate robot without hitting.
        distanceFromWall = new WayPoint(32.86, 83.14436, Math.toRadians(0.0), 0.5, true);

        // The location specific skystone collection values.
        switch(position) {
            case 1:
                // Skystone position 1 specific coordinates
                positionToGrabSkystone1 = new WayPoint(skystoneX, skystone1Y, attackAngle, 1.0, false);
                grabSkystone1 = new WayPoint(skystoneX + 20.0, skystone1Y - 20.0, attackAngle, 1.0, false);
                // RCVS This one doesn't rotate when it pulls back because it hits the wall.
                pullBackSkystone1 = new WayPoint(runLaneX, skystone1Y - 20.0, attackAngle, 0.5, true);
                // Skystone position 4 specific coordinates
                positionToGrabSkystone2 = new WayPoint(skystoneX, skystone4Y, attackAngle, 1.0, false);
                grabSkystone2 = new WayPoint(skystoneX + 20.0, skystone4Y - 20.0, attackAngle, 1.0, false);
                pullBackSkystone2 = new WayPoint(runLaneX, skystone4Y - 20.0, runAngle, 0.5, true);
                break;
            case 2:
                // Skystone position 2 specific coordinates
                positionToGrabSkystone1 = new WayPoint(skystoneX, skystone2Y, attackAngle, 1.0, false);
                grabSkystone1 = new WayPoint(skystoneX + 20.0, skystone2Y - 20.0, attackAngle, 1.0, false);
                pullBackSkystone1 = new WayPoint(runLaneX, skystone2Y - 20.0, runAngle, 0.5, true);
                // Skystone position 5 specific coordinates
                positionToGrabSkystone2 = new WayPoint(skystoneX, skystone5Y, attackAngle, 1.0, false);
                grabSkystone2 = new WayPoint(skystoneX + 20.0, skystone5Y - 20.0, attackAngle, 1.0, false);
                pullBackSkystone2 = new WayPoint(runLaneX, skystone5Y - 20.0, runAngle, 0.5, true);
                break;
            case 3:
                // Skystone position 3 specific coordinates
                positionToGrabSkystone1 = new WayPoint(skystoneX, skystone3Y, attackAngle, 1.0, false);
                grabSkystone1 = new WayPoint(skystoneX + 20.0, skystone3Y - 20.0, attackAngle, 1.0, false);
                pullBackSkystone1 = new WayPoint(runLaneX, skystone3Y - 20.0, runAngle, 0.5, true);
                // Skystone position 6 specific coordinates
                positionToGrabSkystone2 = new WayPoint(skystoneX, skystone6Y, attackAngle, 1.0, false);
                grabSkystone2 = new WayPoint(skystoneX + 20.0, skystone6Y - 20.0, attackAngle, 1.0, false);
                pullBackSkystone2 = new WayPoint(runLaneX, skystone6Y - 20.0, runAngle, 0.5, true);
                break;
        }

        // Get the robot under the bridge to do foundation
        quarryBeforeBridge = new WayPoint(runLaneX, 185.26, runAngle, 0.5, true);
        buildSiteUnderBridge = new WayPoint(runLaneX, 190.8531, runAngle, 1.0, true);

        alignToFoundation = new WayPoint(87.70872, 315.755, Math.toRadians(-180.0), 1.0, false);
        snuggleFoundation = new WayPoint(120.7087, 315.755, Math.toRadians(-180.0), 0.3, false);
        grabFoundation = new WayPoint(126.7087, 315.755, Math.toRadians(-180.0), 0.1, true);
        pullFoundation = new WayPoint(68.37936, 271.206, Math.toRadians(-120.0), 1.0, true);
        // RCVS Moved 10 less in Y because it couldn't hit point.
        pushFoundation = new WayPoint(68.37936, 289.785, runAngle, 0.5, false);
        // false until we figure out if we have to wait for lift.
        buildSiteReadyToRun = new WayPoint(runLaneX, 271.206, runAngle, 1.0, false);
        quarryUnderBridge = new WayPoint(runLaneX, 185.26, runAngle, 1.0, true);
        foundationDeposit = new WayPoint(runLaneX, 289.785, runAngle, 1.0, false);
        park = new WayPoint(runLaneX, 204.3875, runAngle, 1.0, false);

    }
}