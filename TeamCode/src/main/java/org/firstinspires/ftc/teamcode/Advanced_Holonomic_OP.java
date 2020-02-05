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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Advanced Holonomic OP", group = "Iterative Opmode")
@Disabled
public class Advanced_Holonomic_OP extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private BNO055IMU imu;
    private float turnSpeed = 0.5f;

    private boolean speedSwitch = false;


    public float expectedAngle = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double backLeftPower = 0;
        double backRightPower =0;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
       /* double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
        */
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        float angle = (angles.firstAngle);
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        float x = -gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;

        if (gamepad1.a) {
            speedSwitch = true;
        }
        if (gamepad1.b) {
            speedSwitch = false;
        }


        frontLeftPower = y + -x;
        frontRightPower = y + x;
        backLeftPower = y + x;
        backRightPower = y + -x;

        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            int magnitude = (gamepad1.dpad_up ? 1 : 0) + (gamepad1.dpad_down ? -1 : 0);
            frontLeftPower = magnitude * sin;
            frontRightPower = magnitude* cos;
            backLeftPower = magnitude* cos;
            backRightPower = magnitude* sin;
        }
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            int magnitude = (gamepad1.dpad_left ? 1 : 0) + (gamepad1.dpad_right ? -1 : 0);
            frontLeftPower = -magnitude * cos;
            frontRightPower = magnitude* sin;
            backLeftPower = magnitude* sin;
            backRightPower = -magnitude* cos;
        }

        if ( gamepad1.left_bumper){
            frontLeftPower += -turnSpeed;
            frontRightPower += turnSpeed;
            backLeftPower += -turnSpeed;
            backRightPower += turnSpeed;
        }
        else if ( gamepad1.right_bumper) {
            frontLeftPower += turnSpeed;
            frontRightPower += -turnSpeed;
            backLeftPower += turnSpeed;
            backRightPower += -turnSpeed;
        }
        else{
         expectedAngle = angle;
        }

        double max = Math.abs(findMax(frontLeftPower,frontRightPower,backLeftPower,backRightPower));
        double divider;

        if (speedSwitch) {
            divider = 3.5;
        }
        else {
            divider = 1.5;
        }
        if (max != 0) {
            frontLeftPower = (frontLeftPower / max)/ divider;
            frontRightPower = (frontRightPower / max)/ divider;
            backLeftPower = (backLeftPower / max)/ divider;
            backRightPower = (backRightPower / max) /divider;
        }

        leftFrontMotor.setPower(frontLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        leftBackMotor.setPower(backLeftPower);
        rightBackMotor.setPower(backRightPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Motor", " left:(%.2f) right:(%.2f)", frontLeftPower, frontRightPower);
        telemetry.addData("Back Motor", " left:(%.2f) right:(%.2f)", backLeftPower, backRightPower);
        telemetry.addData("Angle", "(%.2f)", angle);
        telemetry.addData("maxPower", "(%.2f)", max);
        telemetry.update();
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private double findMax(double... vals) {
        double max = Double.NEGATIVE_INFINITY;

        for (double d : vals) {
            if (d > max) max = d;
        }

        return max;
    }

}
