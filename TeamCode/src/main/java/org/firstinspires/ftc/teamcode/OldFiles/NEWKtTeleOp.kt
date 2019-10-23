package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.OldFiles.KtRobot

/**
 * Created by KasaiYuki on 11/14/2018.
 */

@TeleOp(name="NEWTeleOp", group="TeleOp")
@Disabled
class NEWKtTeleOp : OpMode()
{
    val robot = KtRobot()

    override fun init() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap)
        robot.lSlideArm?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.flag?.position = 0.8
    }

    /*
        * Code to run ONCE when the driver hits PLAY
    */
    override fun start() {
        robot.lSlideArm?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        try {
            robot.lSlideArm!!.targetPosition = robot.lSlideArm!!.currentPosition
            telemetry.addData("Arm:", "target-(%.2f), curr-(%.2f)", robot.lSlideArm!!.targetPosition, robot.lSlideArm!!.currentPosition)
        }
        catch (n: NullPointerException) {
            telemetry.addData("LSlide pos is NULL!", println(n))
        }
    }

    override fun loop()
    {
        //Tank Drive
        var leftPower: Float = -gamepad1.left_stick_y
        var rightPower: Float = -gamepad1.right_stick_y
        var slidePower: Float = -gamepad2.left_stick_y

        robot.leftDrive?.power = leftPower.toDouble()
        robot.rightDrive?.power = rightPower.toDouble()
        robot.lSlideArm?.power = slidePower.toDouble() //Option 2: use joystick for slide
        //robot.armMot?.power = extPower.toDouble()
        robot.swingFlag(gamepad2)

/*        try {
            if (gamepad2.a)
                robot.dropToken()
            else
                robot.liftToken()

        }
        catch (e: Exception) {
            telemetry.addData("Error in arm!", println(e))
        }*/
        try {
            //curPos = (robot.lSlideArm?.currentPosition).toInt()
            while (robot.lSlideArm!!.currentPosition > robot.lSlideArm!!.targetPosition) {
                try {
                    if (gamepad1.a) {
                        robot.liftRobot(10.0)
                    } else
                        robot.liftRobot(0.0)
                } catch (e: Exception) {
                    telemetry.addData("Error in Linear Slide going up!", println(e))
                }

                try {
                    if (gamepad1.b) {
                        robot.liftRobot(-10.0)
                    } else
                        robot.liftRobot(0.0)
                } catch (e: Exception) {
                    telemetry.addData("Error in Linear Slide going down!", println(e))
                }
            }
        }
        catch (n: NullPointerException) {
            telemetry.addData("Arm is NULL", println(n))
        }

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
        telemetry.addData("Arm Motor", " (%.2f)", -gamepad2.left_stick_y)
        telemetry.addData("Servo", robot.flag?.position)
    }

    override fun stop()
    {

    }

/*  //Will close the arm to different positions based on what is being grabbed
    private fun swingFlag() {
        if (gamepad2.left_bumper) {
            flag?.position = 0.25 //Sphere
        } else if (gamepad2.right_bumper) {
            flag?.position = 0.45 //Cube
        } else {
            flag?.position = 0.0 }
    }*/
}
