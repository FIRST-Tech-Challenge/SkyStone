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

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.ButtonAndEncoderData;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.StoneManipulator.State;

@TeleOp(name="Two Drivers", group="none")
//@Disabled
public class OfficialTeleop extends OpMode {

    private Robot robot;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = Robot.getInstance();
        robot.init(hardwareMap);

        lastDebRead = new boolean[NUMBER_DEBOUNCERS];
        for (int i = 0; i < NUMBER_DEBOUNCERS; i++) {
            lastDebRead[i] = false;
        }
        lastDebTime = new long[NUMBER_DEBOUNCERS];
        for (int i = 0; i < NUMBER_DEBOUNCERS; i++) {
            lastDebTime[i] = Long.MIN_VALUE;
        }

        robot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH, false);
//        robot.stoneManipulator.setExtended(false);
//        robot.stoneManipulator.setGrabbed(false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

//        ButtonAndEncoderData data = ButtonAndEncoderData.getLatest();
//        data.clear();
//        data.addHubData(robot.controlHub);
//        data.addHubData(robot.secondHub);

//        robot.lift.idle();

        telemetry.addData("Status", "Idling Lift Motor...");
        telemetry.update();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

//        ButtonAndEncoderData data = ButtonAndEncoderData.getLatest();
//        data.clear();
//        data.addHubData(robot.controlHub);
//        data.addHubData(robot.secondHub);

        robot.runtime.reset();

//        robot.lift.zero();

//        robot.driveTrain.setLocationZero();

        telemetry.addData("Status", "Lift Initialized.");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

//        ButtonAndEncoderData data = ButtonAndEncoderData.getLatest();
//        data.clear();
//        data.addHubData(robot.controlHub);
//        data.addHubData(robot.secondHub);

        double turn = deadZone(gamepad1.right_stick_x);
        double y = -deadZone(gamepad1.left_stick_y);
        double x = deadZone(gamepad1.left_stick_x);
        robot.driveTrain.spinDrive(x, y, turn);

//        if (robot.stoneManipulator.getState() == State.RESTING) {
//            whenPressedDebounce(() -> gamepad2.dpad_up, () -> robot.lift.moveUp(), 0);
//            whenPressedDebounce(() -> gamepad2.dpad_down, () -> robot.lift.moveDown(), 1);
//        }
//        robot.lift.updatePosition();
//        robot.lift.updateRightMotor();

//        if (robot.lift.getLevel() == 0 && robot.stoneManipulator.getState() == State.RESTING && !robot.stoneManipulator.isGrabbed()) {
            whenPressedDebounce(() -> gamepad2.right_bumper, () -> robot.stoneManipulator.setIntake(1), 2);
//        }
        whenReleasedDebounce(() -> gamepad2.right_bumper, () -> robot.stoneManipulator.setIntake(0), 2);
//        if (robot.stoneManipulator.getState() == State.RESTING && robot.stoneManipulator.isGrabbed()) {
//            whenPressedDebounce(() -> gamepad2.left_bumper, () -> robot.stoneManipulator.setExtended(true), 3);
//        }
//        if (robot.stoneManipulator.getState() == State.EXTENDED) {
//            whenReleasedDebounce(() -> gamepad2.left_bumper, () -> robot.stoneManipulator.setExtended(false), 3);
//        }

//        if (robot.stoneManipulator.getState() != State.INTAKING) {
//            whenPressedDebounce(() -> gamepad2.a, () -> robot.stoneManipulator.setGrabbed(!robot.stoneManipulator.isGrabbed()), 4);
//        }

        whenPressedDebounce(() -> gamepad2.b, () ->
                robot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH, true), 5);
        whenReleasedDebounce(() -> gamepad2.b, () ->
                robot.foundationGrabber.setGrabbed(FoundationGrabber.Hook.BOTH, false), 5);

//        robot.driveTrain.updateLocation();

        telemetry.addData("Drive", "x:%f, y:%f, turn:%f", x, y, turn);
//        telemetry.addData("Lift level", robot.lift.getLevel());
//        telemetry.addData("Drive Base Location", robot.driveTrain.getLocation());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.close();
    }

    // Controller Utilities

    private double deadZone(double original) {
        return Math.abs(original) < 0.12 ? 0 : original;
    }

    private boolean[] lastDebRead;
    private long[] lastDebTime;
    private final int NUMBER_DEBOUNCERS = 10;
    private void whenPressedDebounce(Supplier<Boolean> button, Runnable action, int id) {
        if (button.get() == lastDebRead[id]) {
            return;
        }
        if (robot.runtime.milliseconds() - lastDebTime[id] < 50) {
            return;
        }
        if (button.get()) {
            action.run();
        }
        lastDebRead[id] = button.get();
        lastDebTime[id] = (long) robot.runtime.milliseconds();
    }

    private void whenReleasedDebounce(Supplier<Boolean> button, Runnable action, int id) {
        if (button.get() == lastDebRead[id]) {
            return;
        }
        if (robot.runtime.milliseconds() - lastDebTime[id] < 50) {
            return;
        }
        if (!button.get()) {
            action.run();
        }
        lastDebRead[id] = button.get();
        lastDebTime[id] = (long) robot.runtime.milliseconds();
    }

}
