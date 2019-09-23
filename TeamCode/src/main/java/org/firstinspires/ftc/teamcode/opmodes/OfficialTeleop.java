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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

@TeleOp(name="Two Drivers", group="none")
//@Disabled
public class OfficialTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private ModernRoboticsI2cGyro gyro;
    private MecanumController driveTrain;

    private Servo foundationGrabber;

    private DcMotorEx lift;
    private int liftLevel;
    private int LEVEL_HEIGHT = 300; // ticks per lift level
    private int liftOffset;
    private int lastDpad;
    private long dPadDebounce;
    private final int MAX_LIFT_HEIGHT = 4;

    // sensors and servos

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        MecanumDrive train = new MecanumDriveImpl(leftFront, leftBack, rightFront, rightBack, gyro);
        driveTrain = new MecanumController(train);

        foundationGrabber = hardwareMap.get(Servo.class, "foundation");
        foundationGrabber.scaleRange(0.4, 0.9);
        foundationGrabber.setDirection(Servo.Direction.FORWARD);

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(1, 0, 0.1, 0.2));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        lift.setPower(0.1);

        foundationGrabber.setPosition(0);

        telemetry.addData("Status", "Idling Lift Motor...");
        telemetry.update();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();

        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLevel = 0;
        liftOffset = 0;
        lastDpad = 0;
        dPadDebounce = -50;

        gyro.calibrate();
        driveTrain.zeroDeadReckoner();

        telemetry.addData("Status", "Lift Initialized.");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (driveTrain.isGyroCalibrating()) {
            return;
        }

        double turn = deadZone(-deadZone(gamepad1.left_stick_y) - -deadZone(gamepad1.right_stick_y));
        double y = deadZone(-deadZone(gamepad1.left_stick_y) + -deadZone(gamepad1.right_stick_y)) / 2;
        double x = deadZone(deadZone(gamepad1.left_stick_x) + deadZone(gamepad1.right_stick_x)) / 2;
        driveTrain.spinDrive(x, y, turn, MecanumDrive.TranslTurnMethod.EQUAL_SPEED_RATIOS);

        if (runtime.milliseconds() - dPadDebounce > 50) {
            int dpad = (gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0);
            if (dpad != lastDpad && dpad != 0) {
                liftLevel += dpad;
                if (liftLevel < 0) {
                    liftLevel = 0;
                }
                if (liftLevel > MAX_LIFT_HEIGHT) {
                    liftLevel = MAX_LIFT_HEIGHT;
                }
            }
            lastDpad = dpad;
        }

        lift.setTargetPosition((liftLevel * LEVEL_HEIGHT) + liftOffset);

        foundationGrabber.setPosition(gamepad2.a ? 1 : 0);

        driveTrain.updateLocation();

        telemetry.addData("Drive", "x:%f, y:%f, turn:%f", x, y, turn);
        telemetry.addData("Lift level", liftLevel);
        telemetry.addData("Drive Base Location", driveTrain.getLocation());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private double deadZone(double original) {
        return Math.abs(original) < 0.12 ? 0 : original;
    }

}
