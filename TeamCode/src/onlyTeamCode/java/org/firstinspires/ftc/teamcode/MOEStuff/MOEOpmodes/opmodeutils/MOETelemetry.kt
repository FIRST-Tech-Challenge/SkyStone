package org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.opmodeutils

import org.firstinspires.ftc.robotcontroller.moeglobal.firebase.MOEDatabase
import org.firstinspires.ftc.robotcontroller.moeglobal.firebase.MOEDatabase.firelog
import org.firstinspires.ftc.robotcore.external.Telemetry

class MOETelemetry(val actualTelemetry: Telemetry) {
    init {
        clear()
    }

    private fun clear() {
        firelog.setValue(null)
        actualTelemetry.clearAll()
    }

    fun set(key: String, value: Any) {
        firelog.child(key).setValue(value)
    }


}
