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

import java.util.Timer;
import java.util.TimerTask;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "ADVHOP_ARM", group = "Iterative Opmode")
public class ADVHOP_ARM extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private CRServo grabberCloseServo = null;
    private CRServo grabberRotateServo = null;
    private DcMotor slideTiltMotor = null;
    private DcMotor slideTiltMotor2 = null;
    private CRServo slideExtendServo = null;
    private DcMotorSimple intakeLeftMotor = null;
    private DcMotorSimple intakeRightMotor = null;
    private BNO055IMU imu;
    private float turnSpeed = 0.5f;
    private PIDController pidDrive;
    private double globalAngle, basePower = .30, correction;
    private Orientation lastAngles = new Orientation();
    private boolean speedSwitch = false;
    private boolean turning = false, lastTurning = false;
    private boolean pidActive = false;
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower, max;
    private int numOfRotations;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Starting up..");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        //Grabber servo
        grabberCloseServo = hardwareMap.get(CRServo.class, "grabberCloseServo");
        grabberRotateServo = hardwareMap.get(CRServo.class, "grabberRotateServo");

        slideExtendServo = hardwareMap.get(CRServo.class, "slideExtendServo");
        slideTiltMotor = hardwareMap.get(DcMotor.class, "slideTiltMotor");
        slideTiltMotor2 = hardwareMap.get(DcMotor.class, "slideTiltMotor2");
        slideTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTiltMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeftMotor = hardwareMap.get(DcMotorSimple.class,"intakeLeftMotor");
        intakeRightMotor = hardwareMap.get(DcMotorSimple.class, "intakeRightMotor");
        intakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, basePower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, basePower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double slideExtendPower = gamepad2.left_bumper ? 1 : - gamepad2.left_trigger;
        slideExtendServo.setPower(slideExtendPower);

        double slideTiltPower = gamepad2.left_stick_y / 4;

        if (slideTiltPower == 0) {
            slideTiltPower = 0.05;
        }
        slideTiltMotor.setPower(slideTiltPower);
        slideTiltMotor2.setPower(slideTiltPower);


        double grabberRotatePower = (gamepad2.dpad_up ? 1 : 0) + (gamepad2.dpad_down ? -1 : 0);
        grabberRotateServo.setPower(grabberRotatePower);

        double intakePower = gamepad2.x ? 1 : 0;
        intakeLeftMotor.setPower(intakePower);
        intakeRightMotor.setPower(intakePower);


        // Close grabber
        if (gamepad2.a) {
            grabberCloseServo.setPower(1);
        }
        // Open grabber
        else if (gamepad2.b) {
            Timer timer = new Timer();
            /*timer.schedule(() -> grabberCloseServo.setPower(-1), 1000);*/

            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    grabberCloseServo.setPower(-1);
                }
            }, 1000);
        }

        driveControl();
        update_telemetry();


    }

    private void driveControl() {

        float x = -gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;

        frontLeftPower = y + -x;
        frontRightPower = y + x;
        backLeftPower = y + x;
        backRightPower = y + -x;

        if (gamepad1.right_stick_x < 0) {
            turning = true;
            frontLeftPower += gamepad1.right_stick_x;
            frontRightPower += -gamepad1.right_stick_x;
            backLeftPower += gamepad1.right_stick_x;
            backRightPower += -gamepad1.right_stick_x;
        } else if (gamepad1.right_stick_y > 0) {
            turning = true;
            frontLeftPower += -gamepad1.right_stick_x;
            frontRightPower += gamepad1.right_stick_x;
            backLeftPower += -gamepad1.right_stick_x;
            backRightPower += gamepad1.right_stick_x;
        } else {
            turning = false;
        }

        if (!turning && lastTurning) {
            resetAngle();
        }


        if (!turning) {
            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());
        }
        else{
            correction = 0;
        }

        // Speed controls

        if (gamepad1.x) {
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        numOfRotations = leftBackMotor.getCurrentPosition();

        if (gamepad1.a) {
            speedSwitch = true;
        }
        if (gamepad1.b) {
            speedSwitch = false;
        }

        if (speedSwitch) {
            basePower = .3;
        } else {
            basePower = .6;
        }

        double max = Math.abs(findMax(frontLeftPower, frontRightPower, backLeftPower, backRightPower));

        if (max != 0) {
            frontLeftPower = (frontLeftPower / max) * basePower;
            frontRightPower = (frontRightPower / max) * basePower;
            backLeftPower = (backLeftPower / max) * basePower;
            backRightPower = (backRightPower / max) * basePower;
        }

        if (!pidActive) {
            correction = 0;
        }


        leftFrontMotor.setPower(-(frontLeftPower + correction));
        rightFrontMotor.setPower(-(frontRightPower - correction));
        leftBackMotor.setPower(-(backLeftPower + correction));
        rightBackMotor.setPower(-(backRightPower - correction));

        lastTurning = turning;
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void update_telemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Motor", " left:(%.2f) right:(%.2f)", frontLeftPower, frontRightPower);
        telemetry.addData("Back Motor", " left:(%.2f) right:(%.2f)", backLeftPower, backRightPower);
        telemetry.addData("Imu heading", "(%.2f)", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("maxPower", "(%.2f)", max);
        telemetry.addData("Rotations: ", numOfRotations);
        telemetry.update();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double findMax(double... values) {
        double max = Double.NEGATIVE_INFINITY;

        for (double d : values) {
            if (d > max) max = d;
        }

        return max;
    }

}
