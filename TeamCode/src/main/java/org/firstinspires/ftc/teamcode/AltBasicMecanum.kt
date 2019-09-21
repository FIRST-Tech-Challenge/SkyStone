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

package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime

import kotlin.math.*

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Mecanum Teleop for a four (Mecanum) wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Alt. Basic Mecanum", group = "Linear Opmode")
class AltBasicMecanum : LinearOpMode() {

    // Declare OpMode members.
    private val runtime = ElapsedTime()
    private var leftFront: DcMotor? = null
    private var rightFront: DcMotor? = null
    private var leftRear: DcMotor? = null
    private var rightRear: DcMotor? = null

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get<DcMotor>(DcMotor::class.java, "leftFront")
        rightFront = hardwareMap.get<DcMotor>(DcMotor::class.java, "rightFront")
        leftRear = hardwareMap.get<DcMotor>(DcMotor::class.java, "leftRear")
        rightRear = hardwareMap.get<DcMotor>(DcMotor::class.java, "rightRear")

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront!!.direction = DcMotorSimple.Direction.FORWARD
        rightFront!!.direction = DcMotorSimple.Direction.REVERSE
        leftRear!!.direction = DcMotorSimple.Direction.FORWARD
        rightRear!!.direction = DcMotorSimple.Direction.REVERSE

        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            val Speed = (-gamepad1.left_stick_y).toDouble()
            val Turn = gamepad1.right_stick_x.toDouble()
            val Strafe = -gamepad1.left_stick_x.toDouble()
            holonomic(Speed, Turn, Strafe)
        }
    }

    fun holonomic(Speed: Double, Turn: Double, Strafe: Double) {
        val r = hypot(Strafe, Speed)
        val robotAngle = atan2(Speed, Strafe) - Math.PI / 4
        val leftFrontPower = r * cos(robotAngle) + Turn
        val rightFrontPower = r * sin(robotAngle) - Turn
        val leftRearPower = r * sin(robotAngle) + Turn
        val rightRearPower = r * cos(robotAngle) - Turn

        leftFront!!.power = leftFrontPower
        rightFront!!.power = rightFrontPower
        leftRear!!.power = leftRearPower
        rightRear!!.power = rightRearPower

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: $runtime")
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower)
        telemetry.update()
    }
}
