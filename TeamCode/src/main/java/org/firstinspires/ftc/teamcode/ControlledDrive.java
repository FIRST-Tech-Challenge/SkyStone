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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SQUISH", group="Iterative Opmode")
public class ControlledDrive extends OpMode {

    /*
     * Create position and delta variables inside TeleOp class
     * create DcMotors, controllers, and servos
     */

    private DcMotor leftFront = null;   // white - port 2
    private DcMotor leftRear = null;    // yellow - port 1
    private DcMotor rightFront = null;  // green - port 3
    private DcMotor rightRear = null;   // blue - port 0


    @Override
    public void init() {

        /*
         * Code to run ONCE when the driver hits INIT
         */
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        /*
         * Initialize motors, servos, and controllers with hardwareMap
         */
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // while the power of the wheels is 0, brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void init_loop() {

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */

        /*
         * Set initial servo positions
         */

    }


    @Override
    public void start() {

        /*
         * Code to run ONCE when the driver hits PLAY
         */

    }


    @Override
    public void loop() {

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */

        double control = 2;

        if (gamepad1.right_bumper) {
            control = 4;
        } else if (gamepad1.left_bumper) {
            control = 1;
        }

        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double leftFrontPower = Range.clip((driveY + driveX) + turn, -1.0, 1.0);
        double leftRearPower = Range.clip((driveY - driveX) + turn, -1.0, 1.0);
        double rightFrontPower = Range.clip((driveY - driveX) - turn, -1.0, 1.0);
        double rightRearPower = Range.clip((driveY + driveX) - turn, -1.0, 1.0);

        leftFront.setPower(leftFrontPower / control);
        leftRear.setPower(leftRearPower / control);
        rightFront.setPower(rightFrontPower / control);
        rightRear.setPower(rightRearPower / control);
        /*
         * Set controls on gamepads and update/set position of servos with delta variables
         *
         */

    }


    @Override
    public void stop() {

        /*
         * Code to run ONCE after the driver hits STOP
         */
        // stop all motion
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        // Return claw to normal position

        /*
         * Stop all motion
         */

    }

}
