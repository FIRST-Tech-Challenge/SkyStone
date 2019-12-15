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

package org.firstinspires.ftc.opmodes.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.autonomous.AutonomousRobot;
import org.firstinspires.ftc.robotlib.information.OrientationInfo;
import org.firstinspires.ftc.robotlib.navigation.Point3D;
import org.firstinspires.ftc.robotlib.state.Alliance;
import org.firstinspires.ftc.robotlib.state.ServoState;

class BasicMecanumAutonomous {
    private Alliance alliance;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime elapsedTime = new ElapsedTime();

    private AutonomousRobot robot;

    /**
     * Creates an autonomous mecanum robot
     * @param hardwareMap FTC hardware map
     * @param telemetry FTC Logging
     * @param alliance Alliance to operate under
     */
    BasicMecanumAutonomous(HardwareMap hardwareMap, Telemetry telemetry, Alliance alliance) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;
    }

    /**
     * Initializes the robot
     * Ran before the game starts
     */
    void init() {
        telemetry.addData("Status", "Initialized");
        robot = new AutonomousRobot(this.hardwareMap, alliance, telemetry, elapsedTime);
        robot.init();
    }

    /**
     * Ran after the game starts and before the game loop begins
     */
    void start() {
        elapsedTime.reset();

        // Enable Tracking
        robot.trackables.activate();
    }

    /**
     * Ran once the game has ended
     */
    void end() {
        // Disable Tracking
        robot.trackables.deactivate();
    }

    /**
     * Game Loop Method (runs until stop is requested)
     * @return true - keep looping | false - stop looping
     */
    boolean loop() {
        robot.move(0, 0.7, null, 20);
        robot.move(robot.correctMovement(90), 0.5, null, 34);

        for (int i = 0; i < 2; i++) {
            robot.scanWait(2);

            if (robot.isTrackableVisible() && robot.isSkystoneVisible()) {
                // Get Skystone
                Point3D positionFromSkystone = robot.getPositionFromSkystone();
                Point3D stonePoint3D = new Point3D(robot.getTrackedSkystone().getLocation());
                telemetry.addData("Position relative to Skystone", "{X, Y, Z} = %.0f, %.0f, %.0f", positionFromSkystone.x, positionFromSkystone.y, positionFromSkystone.z);
                robot.move(robot.getCourse(positionFromSkystone, stonePoint3D), 0.3, null, robot.getDistance(positionFromSkystone, stonePoint3D) - 2);
            } else robot.move(0, 0.5, null, 5);

            // Intake Stone
            robot.turn(180, 0.5);
            robot.hardware.updateDeliveryStates(ServoState.FLOOR);
            robot.hardware.blockGrabber.setPosition(ServoState.OPEN);
            robot.wait(2.0);

            robot.hardware.blockGrabber.setPosition(ServoState.CLOSED);
            robot.wait(0.2);
            robot.hardware.updateDeliveryStates(ServoState.CRADLE);
            robot.wait(1.0);
            robot.move(180, 0.5, null, 8);

            // Deliver Stone
            robot.move(robot.correctMovement(-90), 0.5, null, 71);
            robot.move(0, 0.5, null, 14);
            if (i == 0) robot.hardware.updateDeliveryStates(ServoState.ONEBLOCKDEPOSIT);
            else robot.hardware.updateDeliveryStates(ServoState.TWOBLOCKDEPOSIT);
            robot.hardware.blockGrabber.setPosition(ServoState.OPEN);
            robot.wait(2.0);

            // Move to the loading zone
            robot.move(180, 0.5, null, 14);
            robot.move(robot.correctMovement(90), 0.5, null, 94);
        }

        robot.move(robot.correctMovement(90), 0.5, null, 46);
        return false;
    }
}