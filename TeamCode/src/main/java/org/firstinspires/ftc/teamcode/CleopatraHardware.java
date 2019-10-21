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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class CleopatraHardware
{
    /* Public OpMode members. */
    public DcMotor intakeMotorLeft=null;
    public DcMotor intakeMotorRight=null;
    public DcMotor backMotorLeft=null;
    public DcMotor backMotorRight=null;
    public DcMotor frontMotorLeft=null;
    public DcMotor frontMotorRight=null;
    public DcMotor armElbow=null;
    public DcMotor armWrist=null;
    public Servo intakeServoRight=null;
    public Servo intakeServoLeft=null;
    public Servo claw=null;
    public Servo rotator=null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double ELBOW_UP_POWER    =  0.45 ;
    public static final double ELBOW_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public CleopatraHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        intakeMotorLeft  = hwMap.get(DcMotor.class, "intakeMotorLeft");
        intakeMotorRight = hwMap.get(DcMotor.class, "intakeMotorRight");
        backMotorLeft   = hwMap.get(DcMotor.class, "backMotorLeft");
        backMotorRight = hwMap.get(DcMotor.class, "backMotorRight");
        frontMotorLeft = hwMap.get(DcMotor.class, "frontMotorLeft");
        frontMotorRight = hwMap.get(DcMotor.class, "frontMotorRight");
        armElbow = hwMap.get(DcMotor.class,"armElbow");
        armWrist = hwMap.get(DcMotor.class, "armWrist");
        intakeServoRight = hwMap.get(Servo.class, "intakeServoRight");
        intakeServoLeft = hwMap.get(Servo.class, "intakeServoLeft");
        claw = hwMap.get(Servo.class, "claw");
        rotator=hwMap.get(Servo.class, "rotator");


        armElbow.setDirection(DcMotor.Direction.FORWARD);
        armWrist.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorRight.setDirection(DcMotor.Direction.FORWARD);
        backMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        backMotorRight.setDirection(DcMotor.Direction.REVERSE);
        frontMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        frontMotorRight.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        intakeServoRight.setDirection(Servo.Direction.FORWARD);
        intakeServoLeft.setDirection(Servo.Direction.REVERSE);
        rotator.setDirection(Servo.Direction.FORWARD);

        // Set all motors to zero power

        armElbow.setPower(0);
        armWrist.setPower(0);
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
        backMotorLeft.setPower(0);
        backMotorRight.setPower(0);
        frontMotorLeft.setPower(0);
        frontMotorRight.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        armElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armWrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        claw  = hwMap.get(Servo.class, "claw");
        intakeServoLeft = hwMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight= hwMap.get(Servo.class, "intakeServoRight");
        rotator=hwMap.get(Servo.class, "rotator");
        claw.setPosition(MID_SERVO);
        intakeServoLeft.setPosition(MID_SERVO);
        intakeServoRight.setPosition(MID_SERVO);
        rotator.setPosition(MID_SERVO);
    }
 }

