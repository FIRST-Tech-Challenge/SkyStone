package org.firstinspires.ftc.teamcode.test

import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseReference
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOETeleOp
import org.firstinspires.ftc.teamcode.utilities.addData
import org.firstinspires.ftc.teamcode.utilities.get

@TeleOp(name = "FirebaseTest")
class FirebaseTest : MOETeleOp() {
    override fun run() {
        while (opModeIsActive()) {
            loopStuff()
        }
    }

    override fun getCustomRef(ref: DatabaseReference): DatabaseReference? {
        return ref["tests"]["firebase"];
    }

    override fun initOpMode() {
        ref.setValue(5);
        telemetry.addData("testagain")
    }

    private fun loopStuff() {
    }

    override fun onConfigChanged(dataSnapshot: DataSnapshot) {
        val value = dataSnapshot.getValue(Int::class.java)
        telemetry.addData("data", value)
        telemetry.update()

    }
}