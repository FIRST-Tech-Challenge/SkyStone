package org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes

import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.MOEBot

abstract class MOETeleOp : MOEOpMode() {
    final override fun moeInternalInit() {
        robot = MOEBot(this)
    }
}