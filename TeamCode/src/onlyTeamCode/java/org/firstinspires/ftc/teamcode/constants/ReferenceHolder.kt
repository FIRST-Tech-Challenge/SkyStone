package org.firstinspires.ftc.teamcode.constants

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.MOEBot
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOEOpMode


class ReferenceHolder {
    companion object {
        lateinit var moeOpMode: MOEOpMode
        lateinit var hardwareMap: HardwareMap
        lateinit var robot: MOEBot
        fun setRefs(opMode: MOEOpMode) {
            moeOpMode = opMode;
            hardwareMap = opMode.hardwareMap
        }

        fun setRobotRef(robot: MOEBot) {
            this.robot = robot;
        }
    }
}
