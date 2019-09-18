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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.westtorrancerobotics.lib.Angle;
import org.westtorrancerobotics.lib.MecanumController;
import org.westtorrancerobotics.lib.MecanumDrive;

@TeleOp(name="Two Drivers", group="Iterative Opmode")
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
    private MecanumDrive train;

    private DcMotorEx lift;
    private float liftLevel;
    private int lastDpad;
    private long dPadDebounce;
    private final int MAX_LIFT_HEIGHT = 4;

    // sensors and servos

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        train = new MecanumDriveImpl(leftFront, leftBack, rightFront, rightBack, gyro);
        driveTrain = new MecanumController(train);


        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        lastDpad = 0;
        dPadDebounce = -250;

        driveTrain.zeroDeadReckoner();

        telemetry.addData("Status", "Lift Initialized.");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double turn = -gamepad1.left_stick_y - -gamepad1.right_stick_y;
        double y = (-gamepad1.left_stick_y + -gamepad1.right_stick_y) / 2;
        double x = (gamepad1.left_stick_x + gamepad1.right_stick_x) / 2;
        driveTrain.spinDrive(x, y, turn, MecanumDrive.TranslTurnMethod.EQUAL_SPEED_RATIOS);

        if (runtime.milliseconds() - dPadDebounce > 250) {
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

        lift.setTargetPosition((int) (liftLevel * 300));

        driveTrain.updateLocation();

        telemetry.addData("Drive", "x:%f, y:%f, turn:%f", x, y, turn);
        telemetry.addData("Lift level", Math.round(liftLevel));
        telemetry.addData("Drive Base Location", driveTrain.getLocation());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    // TODO replace spinDrive and add the auto methods (to MecanumController)
    // TODO add CONSTANT_BOTH_SPEED to TranslTurnWay enum
    // TODO add sources library with MecanumAttempts class library
    public void spinDrive(Angle ang, double speed, double turn, MecanumDrive.TranslTurnMethod way) {
        double x = ang.getValue(Angle.AngleUnit.RADIANS, Angle.AngleOrientation.COMPASS_HEADING) % 6.283185307179586D;
        x = x < 0.0D ? x + 6.283185307179586D : x;
        double a = Math.cos(x - 0.7853981633974483D);
        double b = Math.cos(x + 0.7853981633974483D);
        double sc = 1.0D;
        double scTurn = 0.0D;
        double turnPwr;
        switch(way) {
            case CONSTANT_TRANSLATION_SPEED:
                sc = speed;
                scTurn = 1 - Math.max(Math.abs(a), Math.abs(b)) * speed;
                break;
            case EQUAL_POWERS:
                sc = speed / (Math.abs(a) + Math.abs(b));
                scTurn = 0.5D;
                break;
            case EQUAL_SPEED_RATIOS:
                double max = Math.max(Math.abs(turn), Math.abs(speed));
                scTurn = Math.abs(turn) * max / (Math.abs(turn) + Math.abs(speed));
                sc = max - scTurn;
                break;
            case CONSTANT_BOTH_SPEED:
                if (Math.abs(speed + turn - 1) > 0.0001) {
                    throw new IllegalArgumentException("Speed and turn must add to one " +
                            "for CONSTANT_BOTH_SPEED drive mode");
                }
                sc = speed;
                scTurn = turn;
                break;
        }
        a *= sc;
        b *= sc;
        turnPwr = turn * scTurn;
        this.train.setMotorPowers(a + turnPwr, b + turnPwr, b - turnPwr, a - turnPwr);
    }

    private long lastLeftFront;
    private long lastRightFront;
    private long lastLeftBack;
    private long lastRightBack;

    private Location location;

    public void zeroDeadReckoner() {
        lastLeftFront = train.getFrontLeftEncoder();
        lastLeftBack = train.getBackLeftEncoder();
        lastRightFront = train.getFrontRightEncoder();
        lastRightBack = train.getBackRightEncoder();
        location = new Location(0, 0, new Angle(0, Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING));
    }

    public void updateLocation() {
        double a = (Math.abs(train.getFrontLeftEncoder() - lastLeftFront) +
                Math.abs(train.getBackRightEncoder() - lastRightBack)) / (2 * Math.sqrt(2));
        double b = (Math.abs(train.getBackLeftEncoder() - lastLeftBack) +
                Math.abs(train.getFrontRightEncoder() - lastRightFront)) / (2 * Math.sqrt(2));
        location.translate(new Point2D(a, new Angle(train.getGyro().getValue(Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING) + 45,
                Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)));
        location.translate(new Point2D(b, new Angle(train.getGyro().getValue(Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING) - 45,
                Angle.AngleUnit.DEGREES, Angle.AngleOrientation.COMPASS_HEADING)));
        location = new Location(location.x, location.y, train.getGyro());
        lastLeftFront = train.getFrontLeftEncoder();
        lastLeftBack = train.getBackLeftEncoder();
        lastRightFront = train.getFrontRightEncoder();
        lastRightBack = train.getBackRightEncoder();
    }

    public Location getLocation() {
        return location;
    }

}
