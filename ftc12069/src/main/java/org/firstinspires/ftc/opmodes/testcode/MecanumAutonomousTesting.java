/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.opmodes.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotlib.autonomous.AutonomousRobot;
import org.firstinspires.ftc.robotlib.information.OrientationInfo;
import org.firstinspires.ftc.robotlib.navigation.Point3D;
import org.firstinspires.ftc.robotlib.state.Alliance;

@Autonomous(name="Mecanum Autonomous TESTING", group="testing")
@Disabled
public class MecanumAutonomousTesting extends LinearOpMode {
    private Alliance alliance;
    private ElapsedTime elapsedTime = new ElapsedTime();

    private AutonomousRobot robot;

    /**
     * Initializes the robot
     * Ran before the game starts
     */
    private void initTest() {
        telemetry.addData("Status", "Initialized");
        alliance = Alliance.BLUE;

        robot = new AutonomousRobot(this.hardwareMap, alliance, telemetry, elapsedTime);
        robot.init();
    }

    /**
     * Ran after the game starts and before the game loop begins
     */
    private void startTest() {
        elapsedTime.reset();

        // Enable Tracking
        robot.trackables.activate();
    }

    /**
     * Ran once the game has ended
     */
    private void endTest() {
        // Disable Tracking
        robot.trackables.deactivate();
    }

    /**
     * Game Loop Method (runs until stop is requested)
     * @return true - keep looping | false - stop looping
     */
    private boolean loopTest() {
        /*if (elapsedTime.seconds() > 25) {
            robot.parkUnderBridge();
            return false;
        }*/

        robot.scan();

        telemetry.addData("Elapsed Time", elapsedTime.seconds() + " seconds");

        // Provide feedback as to where the robot is located (if we know).
        if (robot.isTrackableVisible()) {
            if (robot.isLocationKnown()) {
                // express position (translation) of robot in inches.
                Point3D position = robot.getPosition();
                telemetry.addData("Position (inch)", "{X, Y, Z} = %.1f, %.1f, %.1f", position.x, position.y, position.z);

                // express the orientation of the robot in degrees.
                Orientation orientation = robot.getOrientation();
                telemetry.addData("Orientation (deg)", "{Heading, Roll, Pitch} = %.0f, %.0f, %.0f", orientation.thirdAngle, orientation.firstAngle, orientation.secondAngle);
            }

            telemetry.addData("Visible Target(s)", robot.stringifyVisibleTargets());

            // Move to stone if visible
            VuforiaTrackable trackedStone = robot.getVisibleTrackable("Stone Target");
            if (trackedStone != null) {
                Point3D positionFromSkystone = robot.getPositionFromSkystone();
                Point3D stonePoint3D = new Point3D(trackedStone.getLocation());
                telemetry.addData("Position relative to Skystone", "{X, Y, Z} = %.0f, %.0f, %.0f", positionFromSkystone.x, positionFromSkystone.y, positionFromSkystone.z);
                robot.move(robot.getCourse(positionFromSkystone, stonePoint3D), 0.5, null, robot.getDistance(positionFromSkystone, stonePoint3D));
                //robot.hardware.intakeMotorManager.setMotorsVelocity(1.0);
            }
        } else {
            telemetry.addData("Visible Target", "None");
        }

        /*
            Example Moving/Turning
            robot.simpleMove(robot.getCourseFromRobot(stonePoint3D), 1, 0, robot.getDistanceFromRobot(stonePoint3D));
            robot.turn(90, 0.5);
            robot.move(robot.getCourseFromRobot(stonePoint3D), 1, new OrientationInfo(145, 0.3), robot.getDistanceFromRobot(stonePoint3D));

            Stone Target Debug
            telemetry.addData("Course", Math.toDegrees(robot.getCourse(positionFromSkystone, stonePoint3D)) + " degrees");
            telemetry.addData("Distance", robot.getDistance(positionFromSkystone, stonePoint3D) + " inches");
         */

        telemetry.update();
        return true;
    }

    @Override
    public void runOpMode() {
        // Initialize robot
        this.initTest();

        // Wait for driver to press start
        waitForStart();
        this.startTest();

        // Game Loop
        while (!isStopRequested()) {
            boolean keepLooping = this.loopTest();
            if (!keepLooping) break;
        }

        this.endTest();
    }
}