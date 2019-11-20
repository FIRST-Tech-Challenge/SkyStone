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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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
public class RobotOneHardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  InAndOut     = null;
    //public Servo    leftClaw    = null;
    //public Servo    rightClaw   = null;
    public Servo    leftServo = null;
    public Servo    rightServo = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    //vuforia
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false  ;
    public static final String VUFORIA_KEY =
            "AfSbRRH/////AAABmdZp/meEF0rrnF8x1B4RWA4U4MtiR4e9F4vLUBpFtxRVxfYp7Zn6hqj3zxh3oPeqmJLbXBaQx/bYT2lxN5uViFpcOzCfRx6FRSV3saH8TyKN/IGgOpP5nP0HgYt+zbWlfpo+efBy5YBNjQz0C25jgtKEWVQ/O6ibyOuRL+IZJfLz8dniv1n6lAwa9lLMIrWPOb31OrrcnIzumkoSbr6/0B6mNhzS2YJzkL8gwDNylOuZIcyzuTOCtknO5RpBC86Lfq96P8JKVeQ9h7bkJ9ZXnDm+dLfH+TWsh5wQBRq2rS0PE2Ji9FPn1lfOqk+MjIOHGGaig2jgB75DGhxcF07SHcKYtClXstRrM4L3Cqo3PSZ8";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;
    // Constants for the center support targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;                                 // Units are degrees
    public static final float bridgeRotZ = 180;
    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;
    // Class Members
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    public boolean targetVisible = false;
    public float phoneXRotate    = 0;
    public float phoneYRotate    = 0;
    public float phoneZRotate    = 0;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RobotOneHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftServo = hwMap.get(Servo.class,"leftServo");
        rightServo = hwMap.get(Servo.class,"rightServo");
        leftDrive  = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        InAndOut    = hwMap.get(DcMotor.class, "InAndOut");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        InAndOut.setPower(0);
        leftServo.setPosition(1.0);
        rightServo.setPosition(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        InAndOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        /*leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);*/

        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

    }
 }

