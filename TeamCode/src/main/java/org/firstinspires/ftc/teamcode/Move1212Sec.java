/*
Copyright 2019 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous
@Disabled
public class Move1212Sec extends LinearOpMode {
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_3;
    private DcMotor front_Left_Motor;
    private DcMotor front_Right_Motor;
    private DcMotor rear_Left_Motor;
    private DcMotor rear_Right_Motor;
    private DcMotor frontMotor;
    private Gyroscope imu_1;
    private Gyroscope imu;
    private DcMotor rearMotor;

    private ElapsedTime Time = new ElapsedTime();


    @Override
    public void runOpMode() {
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");
        front_Left_Motor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        front_Right_Motor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rear_Left_Motor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        rear_Right_Motor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");
        frontMotor = hardwareMap.get(DcMotor.class, "frontMotor");
        imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        rearMotor = hardwareMap.get(DcMotor.class, "rearMotor");
        
        front_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        
        front_Left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_Right_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_Left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_Right_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            
            
            for(double power = -1; power < 1; power += 0.1) {

                Time.reset();

                while(Time.seconds() < 10) {

                    front_Left_Motor.setPower(power);
                    front_Right_Motor.setPower(power);
                    rear_Left_Motor.setPower(power);
                    rear_Right_Motor.setPower(power);
                }
            

                RobotLog.d("FL, FR, RL, RR, power , %d , %d , %d , %d , %f", front_Left_Motor.getCurrentPosition(), front_Right_Motor.getCurrentPosition(), rear_Left_Motor.getCurrentPosition(), rear_Right_Motor.getCurrentPosition(), power);

                front_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rear_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rear_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(500);

                front_Left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                front_Right_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rear_Left_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rear_Right_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                /*
                front_Left_Motor.setPower(0);
                front_Right_Motor.setPower(0);
                rear_Left_Motor.setPower(0);
                rear_Right_Motor.setPower(0);
                */

            
            }
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
