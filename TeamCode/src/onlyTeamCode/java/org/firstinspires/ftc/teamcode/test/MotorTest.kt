package org.firstinspires.ftc.teamcode.test

import com.google.firebase.database.DatabaseReference
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOETeleOp
import org.firstinspires.ftc.teamcode.utilities.addData

@TeleOp(name = "MotorTest")
class MotorTest : MOETeleOp() {
    override fun run() {
        while (opModeIsActive()) {
            loopStuff()
        }
    }

    override fun getCustomRef(ref: DatabaseReference): DatabaseReference? {
        return ref
    }

    override fun initOpMode() {
        telemetry.addData("testagain")
        robot.odometry.servos.initServosUp()
    }

    private fun loopStuff() {
        robot.chassis.setPower(FLP = 1.0, FRP = 0.0, BLP = 0.0, BRP = 0.0)
    }
}