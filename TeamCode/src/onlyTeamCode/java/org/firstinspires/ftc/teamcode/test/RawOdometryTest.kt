package org.firstinspires.ftc.teamcode.test

import com.google.firebase.database.DatabaseReference
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOETeleOp
import org.firstinspires.ftc.teamcode.utilities.addData

@TeleOp(name = "RawOdometryTest")
class RawOdometryTest : MOETeleOp() {
    override fun run() {
        while (opModeIsActive()) {
            loopStuff()
        }
    }

    override fun getCustomRef(ref: DatabaseReference): DatabaseReference? {
        return null
    }

    override fun initOpMode() {
        telemetry.addData("test")
        robot.odometry.servos.initServosDown()
    }

    private fun loopStuff() {
//        telemetry.addData()
    }
}