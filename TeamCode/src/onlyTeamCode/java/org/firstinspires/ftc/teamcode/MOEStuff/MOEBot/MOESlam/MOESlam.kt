package org.firstinspires.ftc.teamcode.MOEStuff.MOESlam

import org.firstinspires.ftc.robotcontroller.moeglobal.slam.SlamHandler
import org.firstinspires.ftc.robotcontroller.moeglobal.slam.SlamT265Handler

class MOESlam {
    lateinit var handler: SlamT265Handler;

    init {
        checkConnection()
    }

    private fun checkConnection() {
        val t265Handler = SlamHandler.t265Handler
        handler = t265Handler
        handler.killStream()
        handler.startStream()
    }

    public fun getPose() = handler.curPose
}