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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp(name = "ROneAllOnBot", group = "R1")
//@Disabled
public class RobotOneAll extends LinearOpMode {
    RobotOneHardware robotOne           = new RobotOneHardware();   // Use a Pushbot's hardware
    double left;
    double right;
    double drive;
    double turn;
    double max;
    double armInNew;
    double armOutNew;
    double hooverNew;
    double rDrdv;
    double lDrdv;
    double Irdv;
    double Ordv;
    double Hrdv;
    int I;
    int O;
    int H;

    @Override
    public void runOpMode() {

        robotOne.init(hardwareMap);

        telemetry.addData("Say", "Goonies Never Say Die");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {


            driveIt();
            hoover();
            BringArmIn();
            PutArmOut();
            grab();
            LetMehGo();
            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("armInNew", "%.2f", armInNew);
            telemetry.addData("armOutNew", "%.2f", armOutNew);
            telemetry.addData("HooverNew", "%.2f", hooverNew);
            telemetry.update();



        }


    }

    public void grab() {
        if (gamepad1.left_trigger == 1) {
            //left 1, right 0
            robotOne.leftServo.setPosition(0.77);
            robotOne.rightServo.setPosition(0.22);
        }
    }

    public void LetMehGo() {
        if (gamepad1.right_trigger == 1) {
            //left 1, right 0
            robotOne.leftServo.setPosition(1.0);
            robotOne.rightServo.setPosition(0.0);
        }
    }

    public void driveIt() {
        drive = gamepad1.left_stick_y;
        turn  = -gamepad1.left_stick_x;

        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            right = rDrivetwelveVoltCalc(right);
            left = lDrivetwelveVoltCalc(left);

            left /= max;
            right /= max;
        }

        // Output the safe vales to the motor drives.

        robotOne.leftDrive.setPower(left);
        robotOne.rightDrive.setPower(right);
    }

    public void hoover() {
        if (gamepad1.x) {
            //.25 and .07 hover low no block
            //.30 and .05 hover medium no block
            //.35 and 0 straight up no block
            hooverNew = HtwelveVoltCalc(0.50);
            //System.out.println("new power from hoover .1: " + newPower);
            robotOne.InAndOut.setPower(hooverNew);
            sleep(900);
            hooverNew = HtwelveVoltCalc(0.05);
            robotOne.InAndOut.setPower(hooverNew);
            //robotOne.InAndOut.setPower(0);
            H=0;
            while (H == 0) {
                if(gamepad1.b) {
                    H++;
                }
                driveIt();
            }
            while (H < 120) {
                robotOne.InAndOut.setPower(0);
                sleep(5);
                hooverNew = HtwelveVoltCalc(.06);
                robotOne.InAndOut.setPower(hooverNew);
                sleep(15);
                H++;
                telemetry.addData("H: ", H);
            }
            robotOne.InAndOut.setPower(0);
        }
    }

    public void BringArmIn() {
        if (gamepad1.a) {
            armInNew = IntwelveVoltCalc(0.5);
            robotOne.InAndOut.setPower(armInNew);
            sleep(725);
            armInNew = IntwelveVoltCalc(0.1);
            robotOne.InAndOut.setPower(armInNew);
            sleep(350);
            robotOne.InAndOut.setPower(0);
            I=0;
            while (I < 95) {
                robotOne.InAndOut.setPower(0);
                sleep(5);
                armInNew = IntwelveVoltCalc(-.06);
                robotOne.InAndOut.setPower(armInNew);
                sleep(15);
                I++;
                telemetry.addData("I: ", I);
            }
            robotOne.InAndOut.setPower(0);
        }
    }


    public void PutArmOut() {
        if (gamepad1.y) {
            armOutNew = OuttwelveVoltCalc(-0.5);
            robotOne.InAndOut.setPower(armOutNew);
            sleep(700);
            armOutNew = OuttwelveVoltCalc(-0.3);
            robotOne.InAndOut.setPower(armOutNew);
            sleep(250);
            armOutNew = OuttwelveVoltCalc(-0.1);
            robotOne.InAndOut.setPower(armOutNew);
            sleep(100);
            //armOutNew = OuttwelveVoltCalc(-0.1);
            //robotOne.InAndOut.setPower(armOutNew);
            //sleep(800);
            O=0;
            while (O < 100) {
                robotOne.InAndOut.setPower(0);
                sleep(3);
                armOutNew = OuttwelveVoltCalc(.09);
                robotOne.InAndOut.setPower(armOutNew);
                sleep(15);
                O++;
            }
            robotOne.InAndOut.setPower(0);
        }
    }

    public double rDrivetwelveVoltCalc(double y) {
        rDrdv = getBatteryVoltage();
        double rDrivePower = (12 / rDrdv) * y;
        return rDrivePower;
    }

    public double lDrivetwelveVoltCalc(double y) {
        lDrdv = getBatteryVoltage();
        double lDrivePower = (12 / lDrdv) * y;
        return lDrivePower;
    }

    public double IntwelveVoltCalc(double y) {
        Irdv = getBatteryVoltage();
        double InPower = (12 / Irdv) * y;
        return InPower;
    }

    public double OuttwelveVoltCalc(double y) {
        Ordv = getBatteryVoltage();
        double OutPower = (12 / Ordv) * y;
        return OutPower;
    }

    public double HtwelveVoltCalc(double y) {
        Hrdv = getBatteryVoltage();
        double HPower = (12 / Hrdv) * y;
        return HPower;
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
