package org.firstinspires.ftc.teamcode.OldFiles

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by KasaiYuki on 9/20/2018.
 */

@TeleOp(name="KtTeleOp", group="TeleOp")
@Disabled
class KtTeleOp : OpMode()
{
    //using robot class for motors, servos etc
    val robot = KtRobot()
    var slowDown = 1.25//default
    var touched = false
    override fun init()
    {
        telemetry.addData("Status", "Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap)

    }

    override fun start() {
        robot.flag?.position = 0.15
    }

    override fun loop()
    {
        slowDown = if(gamepad1.left_bumper) 2.0 else 1.25 //condensed if else

        //Tank Drive
        var leftPower: Float = -gamepad1.left_stick_y
        var rightPower: Float = -gamepad1.right_stick_y

        robot.leftDrive?.power = leftPower.toDouble() / slowDown
        robot.rightDrive?.power = rightPower.toDouble() / slowDown
        //robot.armMot?.power = extPower.toDouble()
        robot.swingFlag(gamepad1)

        if(!robot.touch!!.state) {
            touched = true
            telemetry.addData("Touch sensor:", "pressed in if")
            telemetry.update()
        }
        else if(robot.touch!!.state)
        {
            touched = false
        }

        try {
            if(gamepad1.a && !touched) {
                robot.liftRobot(10.0)//up
            }
            else if(gamepad1.a && touched) {
                telemetry.addData("Touch sensor:", "pressed")
                telemetry.update()
            }

/*            else
                robot.liftRobot(0.0)*/
        }
        catch (e: Exception) {
            telemetry.addData("Error in Linear Slide going up!", println(e))
        }

        try {
            if(gamepad1.b) {
                robot.liftRobot(-10.0)
            }
            else
                robot.liftRobot(0.0)
        }
        catch (e: Exception) {
            telemetry.addData("Error in Linear Slide going down!", println(e))
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