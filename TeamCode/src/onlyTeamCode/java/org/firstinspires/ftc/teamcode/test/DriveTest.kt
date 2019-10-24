package org.firstinspires.ftc.teamcode.test

import com.google.firebase.database.DatabaseReference
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOETeleOp
import org.firstinspires.ftc.teamcode.utilities.addData

@TeleOp(name = "DriveTest")
class DriveTest : MOETeleOp() {
    override fun run() {
        while (opModeIsActive()) {
            loopStuff()
        }
    }


    override fun initOpMode() {
        telemetry.addData("testagain")
        robot.odometry.servos.initServosUp()
    }

    private fun loopStuff() {
        val rawY = (-gamepad1.left_stick_y).toDouble()
        val rawX = gamepad1.left_stick_x.toDouble()

        var rot = gamepad1.right_stick_x.toDouble()


        var fwd = rawY
        fwd *= fwd * fwd
        var FRP = fwd - rawX - rot
        var FLP = fwd + rawX + rot
        var BRP = fwd + rawX - rot
        var BLP = fwd - rawX + rot

        var max = if (FRP > 1) FRP else 1.0
        if (max < FRP) {
            max = FRP
        }
        if (max < BLP) {
            max = BLP
        }
        if (max < BRP) {
            max = BRP
        }
        if (max < FLP) {
            max = FLP
        }

        FLP /= max
        FRP /= max
        BLP /= max
        BRP /= max

        robot.chassis.setPower(FLP, FRP, BLP, BRP)
    }
}