package org.firstinspires.ftc.teamcode.test

import com.google.firebase.database.DatabaseReference
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOETeleOp
import org.firstinspires.ftc.teamcode.utilities.addData

@TeleOp(name = "ControllerTest")
class ControllerTest : MOETeleOp() {
    override fun run() {
        while (opModeIsActive()) {
            loopStuff()
        }
    }


    override fun initOpMode() {
        telemetry.addData("testagain")
    }

    private fun loopStuff() {
        mainGamepad.onButton("A") {
            telemetry.addData(it)
        }
    }

}