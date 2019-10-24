package org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.opmodeutils

import android.util.Log
import com.qualcomm.robotcore.hardware.Gamepad


class MOEGamePad(private val gamepad: Gamepad) {
    private var callbackEnabled: Boolean = false
    fun onButton(button: String, listener: (state: Boolean) -> Unit) {
        enableCallback()
    }

    private fun enableCallback() {
        callbackEnabled = true

        reflectCallback()
        Log.e("done", "done")
    }

    private fun reflectCallback() {

    }

}