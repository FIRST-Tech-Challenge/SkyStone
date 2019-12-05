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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "RED_Platform_Bridge_Block2", group = "Linear Opmode")
@Disabled
public class RED_Platform_Bridge_Block2 extends BaseAutoOpMode {


    double                  globalAngle, power = 1, correction;

    int startingSide = -1;  //Set to 1 for blue and -1 for Red


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        GetHardware();
        GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        front_left.setPower(1 * startingSide);
        rear_left.setPower(-.5 * startingSide);
        front_right.setPower(-.5 * startingSide);
        rear_right.setPower(.5 * startingSide);
        //sleep(250);
        sleep(725);
        CutMotors();

        // Larson added this. This is added to have the robot move back to the wall to hopefully straighten it.
        RunAllMotorsBackward();
        sleep(100);

        UnfoldRobot();
        resetAngle();
//Clamps Half
        Clamp_Left.setPosition(0.5);
        Clamp_Right.setPosition(0.5);
        sleep(1000);
//Drive forward
        RunAllMotors();
        sleep(250);
        CutMotors();
//clamps down
        Clamp_Left.setPosition(1);
        Clamp_Right.setPosition(0f);
        sleep(1000);
//drive backward
        RunAllMotorsBackward();
        sleep(600);

//strafe right
        front_left.setPower(-1 * startingSide);
       rear_left.setPower(.7 * startingSide);
        front_right.setPower(1 * startingSide);
        rear_right.setPower(-1 * startingSide);
        sleep(1400);
        CutMotors();
        //sleep(600);

        //front_left.setPower(1);
        //rear_left.setPower(1);
        //front_right.setPower(-1);
        //rear_right.setPower(-1);
        //sleep(1900);

        // Turn towards right
        rotate(55 * startingSide, 1);

        // Robot stops
        RunAllMotors();
        resetAngle();
        sleep(700);
        CutMotors();

        //End of moving platform

        // Clamps go up
        Clamp_Left.setPosition(0f);
        Clamp_Right.setPosition(1f);
        sleep(1000);

        // Servo is pushed out
        Release_Servo.setPosition(0.4);
        sleep(1000);

        //Moves pulley system back
        top_motor.setPower(-1);
        sleep(200);

        //Controls crane movement to stop
        while(Top_Sensor_Rear.getState())
        {
            top_motor.setPower(1);
            telemetry.addData("Loop Crane " , Top_Sensor_Rear.getState());
            telemetry.update();
        }

        top_motor.setPower(0);

        //Controls crane movement to go
        while(bottom_touch.getState())
        {
            lift_left.setPower(1);
            lift_right.setPower(1);
            telemetry.addData("Loop Lift " , bottom_touch.getState());
            telemetry.update();
        }
        telemetry.addData("Loop Lift ", "Out Of Loop");
        telemetry.update();

        //top_motor.setPower(0);


      //  lift_left.setPower(1);
      //  lift_right.setPower(1);
      //  sleep(1500);

        //The lift motors are stopped
        lift_left.setPower(0);
        lift_right.setPower(0);

        //Robot moves backward
        RunAllMotorsBackward();
        sleep(2500);
        CutMotors();

        //End Of Bridge, Start Of Block


        //rotates robot to the left
        rotate(270, -1);

        //lift is moved down
        lift_left.setPower(-1);
        lift_right.setPower(-1);
        sleep(150);

        //controls lift movement to stop
        while(Top_Sensor_Front.getState())
        {
            top_motor.setPower(-0.7);
            telemetry.addData("Loop Crane " , Top_Sensor_Front.getState());
            telemetry.update();
        }
        top_motor.setPower(0);

        //clamps moved down
        Clamp_Left.setPosition(0.7);
        Clamp_Right.setPosition(0.3);
        sleep(1000);

        //Robot is stopped
        RunAllMotors();
        sleep(200);
        CutMotors();

        //Lift goes up
        lift_left.setPower(1);
        lift_right.setPower(1);
        sleep(150);

        //Robot moves backwards
        RunAllMotorsBackward();
        sleep(250);
        CutMotors();

        //Turns to the left
        rotate(-89, 1);

        //Robots stops
        RunAllMotors();
        sleep(700);
        CutMotors();

        //Robot moves backwards
        RunAllMotorsBackward();
        sleep(100);
        CutMotors();










    }


}

