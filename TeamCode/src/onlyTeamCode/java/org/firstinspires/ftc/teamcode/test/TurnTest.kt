package org.firstinspires.ftc.teamcode.test

import com.google.firebase.database.DatabaseReference
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOETeleOp
import org.firstinspires.ftc.teamcode.utilities.addData

@TeleOp(name = "TurnTest")
class TurnTest : MOETeleOp() {
    override fun run() {
        while (opModeIsActive()) {
            loopStuff()
        }
    }

    override fun getCustomRef(ref: DatabaseReference): DatabaseReference? {
        return null
    }

    override fun initOpMode() {
        telemetry.addData("testagain")
        robot.odometry.servos.initServosUp()
    }

    private fun loopStuff() {

        robot.chassis.turnPower(gamepad1.left_stick_x.toDouble())
    }
}