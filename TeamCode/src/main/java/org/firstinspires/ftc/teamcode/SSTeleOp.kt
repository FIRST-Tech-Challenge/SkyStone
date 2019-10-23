package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by KasaiYuki on 9/20/2018.
 */

@TeleOp(name="SSTeleOp", group="TeleOp")
//@Disabled
class SSTeleOp : OpMode() {
    //using robot class for motors, servos etc
    val robot = SSRobot()
    val zero = 0.0.toFloat()
    var slowDown = 1//default
    var touched = true
    var linSlidePow:Float = 0.00.toFloat()

    override fun init() {
        telemetry.addData("Status: ", "TeleOp Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap)

    }

    override fun start() {

    }

    override fun loop() {
        //slowDown = if(gamepad1.left_bumper) 2.0 else 1.25 //condensed if else

        //Tank Drive-sets power equal to numerical value of joystick positions
        var leftPower: Float = -gamepad1.left_stick_y
        var rightPower: Float = -gamepad1.right_stick_y

        robot.leftDrive?.power = leftPower.toDouble() / slowDown
        robot.rightDrive?.power = rightPower.toDouble() / slowDown
        if(!robot.touch!!.state) {
            touched = true
            telemetry.addData("Touch sensor:", "pressed in if")
            telemetry.update()
        }
        else if(robot.touch!!.state)
        {
            touched = false
        }

        if(!touched) {
            linSlidePow = when {
                gamepad1.left_trigger != zero -> (-1 * gamepad1.left_trigger)
                else -> if (gamepad1.right_trigger != zero) (gamepad1.right_trigger)
                else (zero)
            }
        }

        robot.liftSlideY(linSlidePow)

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
        telemetry.addData("Arm Motor", " (%.2f)", -gamepad2.left_stick_y)
    }

    override fun stop()
    {
        telemetry.addData("Status: ", "TeleOp Terminated")
        telemetry.update()
    }
}
