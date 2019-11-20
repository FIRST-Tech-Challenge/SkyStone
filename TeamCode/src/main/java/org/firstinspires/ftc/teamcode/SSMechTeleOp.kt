package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.Range

/**
 * Created by KasaiYuki on 9/20/2018.
 */

@TeleOp(name = "SSMechTeleOp", group = "TeleOp")
//@Disabled
class SSMechTeleOp : OpMode() {
    //using robot class for motors, servos etc
    val robot = SSMechRobot()
    val zero = 0.0.toFloat()
    var slowDown = 1.85//default
    var tooHigh = true //if v slide is too high
    var tooLow = true //if v slide is too low
    var touched = false //if touch sensor is pressed
    var slideP = 0.5 //h slide postion
    var linSlidePow: Float = 0.00.toFloat() //v slide power
    var curPos = 0
    var leftPower: Float = 0.0.toFloat()
    var rightPower: Float = 0.0.toFloat()
    val max = 1860
    var strafePow: Double = 0.00
    var drive = 0.0
    var turn = 0.0


    override fun init() {
        telemetry.addData("Status: ", "TeleOp Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap)
        robot.vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER //Use encoders for linear slide motor
        curPos = robot.vSlide!!.currentPosition

    }

    override fun start() { //runs once when play button is pushed
        robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    override fun loop() {

        /**
         * Gamepad1: Tank Drive-Left Stick y=Left Motor; right stick y=Right Motor
         * Gamepad2: Crane: Right Stick Y = Y Slide; Left Bumper = pinch claw; Left Stick Y= X Slide; a = hook
         */

        touched = !robot.touch!!.state //true if not pressed
        slowDown = if (gamepad1.left_bumper) 2.35 else 1.00 //condensed if else

        if (gamepad1.a) tankMode()
        else if (gamepad1.b) povMode()
        else povMode()

        //Vertical Slide Power Calculation
        linSlidePow = gamepad2.right_stick_y //negative for up
        linSlidePow = when { //when pos is too low and stick is negative, do nothing; same for too high and positive
            tooLow and (linSlidePow > 0) -> 0.toFloat()
            tooHigh and (linSlidePow < 0) -> 0.toFloat()
            else -> gamepad2.right_stick_y
        }
        tooHigh = curPos >= max
        tooLow = curPos < 0

        //if (curPos > 1500) linSlidePow /= 1.2.toFloat() //slow slide if greater than value
        robot.vSlide?.power = -linSlidePow.toDouble()//controls vertical slide, flips sign
        slideP = (gamepad2.left_stick_y.toDouble() / 2) + 0.5 // horizontal slide
        if (touched) { //controls horizontal slide with the left stick of gp2
            if (slideP > 0.5) robot.hSlide?.position = slideP //1=back; 0=forward
            else robot.hSlide?.position = 0.5
        } else robot.hSlide?.position = slideP
        robot.pinch(gamepad2) //operates claw
        curPos = robot.vSlide!!.currentPosition
        robot.dropHook(gamepad2)

        if (touched) telemetry.addData("Touch Sensor:", "Activated")
        if (tooHigh) telemetry.addData("Linear Slide Y Error:", "MAX HEIGHT REACHED")
        if (tooLow) telemetry.addData("Linear Slide Y Error:", "MIN HEIGHT REACHED")
        if (gamepad1.left_bumper) telemetry.addData("Slowdown:", "Engaged!")
        telemetry.addData("Motors: left = $leftPower, right = $rightPower, pow = $linSlidePow", "") //kotlin string templates
        telemetry.addData("Attachments:", "HSlide = ${robot.hSlide?.position?.toFloat()}, " +
                "Claw = ${robot.claw?.position?.toFloat()}, " +
                "VSlide = ${curPos.toFloat()}", "")
    }

    override fun stop() {
        robot.brake()
        telemetry.addData("Status: ", "TeleOp Terminated")
        telemetry.update()
    }

    fun tankMode() {
        //Tank Drive-sets power equal to numerical value of joystick positions
        leftPower = -gamepad1.left_stick_y
        rightPower = -gamepad1.right_stick_y
        robot.leftPow(leftPower.toDouble() / slowDown)
        robot.rightPow(rightPower.toDouble() / slowDown)
        strafePow = when {
            gamepad1.left_trigger > 0 -> gamepad1.left_trigger.toDouble()
            gamepad1.right_trigger > 0 -> -gamepad1.right_trigger.toDouble()
            else -> 0.00
        }
        robot.strafe(strafePow)
    }

    fun povMode() {
        //POV Mode-left joystick=power(y power and strafing), right joystick=turning
        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)


        var drive = (-gamepad1.left_stick_y).toDouble()
        var turn = gamepad1.left_stick_x.toDouble() * 1.5
        var strafe = gamepad1.right_stick_x.toDouble()

        var frontLeftPower = (drive + turn + strafe)
        var backLeftPower = (drive - turn + strafe)
        var frontRightPower = (drive - turn - strafe)
        var backRightPower = (drive + turn - strafe)

        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) { //normalizing values to [-1.0,1.0]
            // Find the largest power
            var max = 0.0
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower))
            max = Math.max(Math.abs(frontRightPower), max)
            max = Math.max(Math.abs(backRightPower), max)

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            robot.fLDrive?.power = frontLeftPower / max
            robot.bLDrive?.power = backLeftPower / max
            robot.fRDrive?.power = frontRightPower / max
            robot.bRDrive?.power = backRightPower / max
        }

    }
}
