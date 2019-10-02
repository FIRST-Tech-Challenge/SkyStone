package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by KasaiYuki on 9/20/2018.
 */

@TeleOp(name="SSTeleOp", group="TeleOp")
@Disabled
class SSTeleOp : OpMode() {
    //using robot class for motors, servos etc
    val robot = SSRobot()
    var slowDown = 1//default

    override fun init() {
        telemetry.addData("Status", "TeleOp Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap)

    }

    override fun start() {

    }

    override fun loop() {
        //slowDown = if(gamepad1.left_bumper) 2.0 else 1.25 //condensed if else

        //Tank Drive
        var leftPower: Float = -gamepad1.left_stick_y
        var rightPower: Float = -gamepad1.right_stick_y

        robot.leftDrive?.power = leftPower.toDouble() / slowDown
        robot.rightDrive?.power = rightPower.toDouble() / slowDown
        //robot.armMot?.power = extPower.toDouble()

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
