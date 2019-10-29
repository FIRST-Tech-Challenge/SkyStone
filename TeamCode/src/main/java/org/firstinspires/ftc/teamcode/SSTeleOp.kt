package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

/**
 * Created by KasaiYuki on 9/20/2018.
 */

@TeleOp(name = "SSTeleOp", group = "TeleOp")
//@Disabled
class SSTeleOp : OpMode() {
    //using robot class for motors, servos etc
    val robot = SSRobot()
    val zero = 0.0.toFloat()
    var slowDown = 1//default
    var tooHigh = true
    var tooLow = true
    var slideP = 0.5
    var linSlidePow: Float = 0.00.toFloat()
    var curPos = 0
    var leftPower: Float = 0.0.toFloat()
    var rightPower: Float = 0.0.toFloat()
    val max = 1880


    override fun init() {
        telemetry.addData("Status: ", "TeleOp Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap)
        robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //Use encoders for linear slide motor
        curPos = robot.vSlide!!.currentPosition

    }

    override fun start() { //runs once when play button is pushed
        robot.vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.hSlide?.position = 0.7
    }

    override fun loop() {
        /**
         * Gamepad1: Tank Drive-Left Stick y=Left Motor; right stick y=Right Motor
         * Gamepad2: Crane: Left Trigger=lower V Slide; Right Trigger=raise V Slide; A=pinch claw; Left Stick Y= X Slide
         */

        //slowDown = if(gamepad1.left_bumper) 2.0 else 1.25 //condensed if else
        //Tank Drive-sets power equal to numerical value of joystick positions

        leftPower = -gamepad1.left_stick_y
        rightPower = -gamepad1.right_stick_y

        robot.leftDrive?.power = leftPower.toDouble() / slowDown
        robot.rightDrive?.power = rightPower.toDouble() / slowDown


        try {
            linSlidePow = when { //when left trigger pressed, if too low is false, slide can move; same for right trigger and too high
                (gamepad2.left_trigger != zero) and !tooLow -> -1 * gamepad2.left_trigger //goes down b/c negative
                (gamepad2.right_trigger != zero) and !tooHigh -> gamepad2.right_trigger //goes up
                else -> zero
            }
        } catch (e: Exception) { telemetry.addData("Linear Slide Y Calculation Error:", println(e)) }

        try {
            tooHigh = curPos >= max
            tooLow = curPos < 0
        } catch (e: Exception) { telemetry.addData("tooHigh/tooLow Error:", println(e)) }

        if (tooHigh) telemetry.addData("Linear Slide Y Error:", "MAX HEIGHT REACHED")
        if (tooLow) telemetry.addData("Linear Slide Y Error:", "MIN HEIGHT REACHED")

        try {
            slideP = (gamepad2.left_stick_y.toDouble() / 2) + 0.5
            robot.hSlide?.position = slideP //controls horizontal slide with the left stick of gp2
            robot.pinch(gamepad2) //operates claw
            robot.liftSlideY(linSlidePow)//controls vertical slide
            curPos = robot.vSlide!!.currentPosition
        } catch (e: Exception) { telemetry.addData("Movement Error:", println(e)) }

        telemetry.addLine()
                .addData("Servo:", "HSlide(%.2f), Claw(%.2f)", robot.hSlide?.position?.toFloat(),
                        robot.claw?.position?.toFloat())
                .addData("Motors: left = $leftPower, right = $rightPower", 0) //kotlin string templates
                .addData("Linear Slide Position: ${curPos.toFloat()}", 0)
    }

    override fun stop() {
        robot.brake()
        telemetry.addData("Status: ", "TeleOp Terminated")
        telemetry.update()
    }
}
