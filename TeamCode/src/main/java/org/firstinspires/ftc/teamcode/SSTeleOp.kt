package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

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
    var tooHigh = true
    var tooLow = true
    var linSlidePow:Float = 0.00.toFloat()
    var rot = 0
    val max = 1780


    override fun init() {
        telemetry.addData("Status: ", "TeleOp Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap, true)
        robot.linSlideY?.mode=DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rot = robot.linSlideY!!.currentPosition

    }

    override fun init_loop(){
        robot.linSlideY?.mode=DcMotor.RunMode.RUN_USING_ENCODER
    }

    override fun loop() {
        //slowDown = if(gamepad1.left_bumper) 2.0 else 1.25 //condensed if else

        //Tank Drive-sets power equal to numerical value of joystick positions
        var leftPower: Float = -gamepad1.left_stick_y
        var rightPower: Float = -gamepad1.right_stick_y

        robot.leftDrive?.power = leftPower.toDouble() / slowDown
        robot.rightDrive?.power = rightPower.toDouble() / slowDown

        if(true) {
            if ((gamepad1.left_trigger != zero) and !tooLow)
                linSlidePow = -1 * gamepad1.left_trigger
            else if ((gamepad1.right_trigger != zero) and !tooHigh)
                linSlidePow = gamepad1.right_trigger
            else
                linSlidePow = zero
        }
        else
            telemetry.addData("Linear Slide ", "MAX/MIN HEIGHT REACHED")



        robot.liftSlideY(linSlidePow)
        rot = robot.linSlideY!!.currentPosition

        if(rot > max)
            tooHigh = true
        if (rot < max)
            tooHigh = false
        if (rot > 0)
            tooLow = false
        if(rot < 0)
            tooLow = true

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
        telemetry.addData("Arm Motor", " (%.2f)", -gamepad2.left_stick_y)
        telemetry.addData("Linear Slide ", "Position: (%.2f)", rot.toFloat())
    }

    override fun stop()
    {
        telemetry.addData("Status: ", "TeleOp Terminated")
        telemetry.update()
    }
}
