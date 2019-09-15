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

package org.eastsideprep.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "15203: Teleop Simple", group = "15203")

public class Simple15203 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware15203 robot = new Hardware15203();

    @Override
    public void runOpMode() {
       /* double left;
        double right;*/
        boolean armU;
        boolean armD;
        boolean hoist;
        boolean release;




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Peter");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        //robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            float x = gamepad1.left_stick_x;
            float y = -gamepad1.left_stick_y; // Negate to get +y forward.
            float rotation = -gamepad1.right_stick_x;
            float speedControl = 0.5f * (1.0f + gamepad1.left_trigger);
            double biggestControl = Math.sqrt(x * x + y * y);
            double biggestWithRotation = Math.sqrt(x * x + y * y + rotation * rotation);

            double angle = Math.atan2(y, -x) - Math.PI / 2.0;

            double[] powers = robot.getDrivePowersFromAngle(angle);
            double pow2 = 0.0;
            for (int i = 0; i < robot.allMotors.length; i++) {
                double pow = powers[i] * biggestControl + rotation * robot.rotationArray[i];
                powers[i] = pow;
                pow2 += pow * pow;
            }

            if (biggestWithRotation != 0.0) {
                double scale = Math.sqrt(pow2);
                for (int i = 0; i < robot.allMotors.length; i++) {
                    robot.allMotors[i].setPower(
                            powers[i] / scale * biggestWithRotation * speedControl);
                }
            } else {
                for (int i = 0; i < robot.allMotors.length; i++)
                    robot.allMotors[i].setPower(0.0);
            }

            armU = gamepad1.dpad_up;
            armD = gamepad1.dpad_down;
            hoist = gamepad1.x;
            release = gamepad1.b;



/*
            if (!sd_changed && gamepad1.left_bumper) {
                slowdrive = !slowdrive;
                sd_changed = true;
            } else {
                sd_changed = false;
            }


*/


            if (hoist) {
                robot.armMotor.setPower(-1.0);
            } else if (release) {
                robot.armMotor.setPower(1.0);
            } else if (armU) {
                robot.armMotor.setPower(0.2);
            } else if (armD) {
                robot.armMotor.setPower(-0.2);
            } else {
                robot.armMotor.setPower(0.0);
            }


//            //TODO: Servos
//            // Use gamepad Y & A raise and lower the arm
//            if (gamepad1.a) {
//                whackerPosition = 0;
//            }
//            else if (gamepad1.y) {
//                whackerPosition = 0.4;
//            }
//            if(gamepad1.x){
//                leftArmPosition -= 0.1;
//                rightArmPosition +=0.1;
//            } else if (gamepad1.b){
//                leftArmPosition += 0.1;
//                rightArmPosition -=0.1;
//            }
//            if(gamepad1.left_bumper){
//                position = robot.lifter.getCurrentPosition();
//            }
//
////
////            // Use gamepad X & B to open and close the claw
////            if (gamepad91.x)
////                clawPosition += CLAW_SPEED;
////            else if (gamepad1.b)
////                clawPosition -= CLAW_SPEED;
////
////            // Move both servos to new position.
//            // at 0 right is all the way in and left is all the way out
//            whackerPosition  = Range.clip(whackerPosition, 0.0, 0.6);
//            rightArmPosition = Range.clip(rightArmPosition, 0.3, 0.8);
//            leftArmPosition = Range.clip(leftArmPosition, 0.3,0.8);
//            robot.whacker.setPosition(whackerPosition);
//            robot.leftArm.setPosition(leftArmPosition);
//            robot.rightArm.setPosition(rightArmPosition);
////            robot.arm.setPosition(armPosition);
////            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
////            robot.claw.setPosition(clawPosition);
//


            // Send telemetry message to signify robot running;
            telemetry.addLine()
//            .addData("left", "%.2f", left)
//            .addData("right", "%.2f", right)
                    .addData("arm up", "%s", armU?"on":"off")
                    .addData("arm down", "%s", armD?"on":"off");
            telemetry.addLine();
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);

        }
    }
}
