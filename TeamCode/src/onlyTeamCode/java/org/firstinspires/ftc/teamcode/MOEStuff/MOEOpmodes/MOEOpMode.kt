package org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes

import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseReference
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcontroller.moeglobal.firebase.MOEConfig
import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.MOEBot
import org.firstinspires.ftc.teamcode.MOEStuff.MOEFirebase.MOEEventListener
import org.firstinspires.ftc.teamcode.MOEStuff.MOEFirebase.MOEFirebase
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.opmodeutils.MOEGamePad
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.opmodeutils.MOETelemetry
import org.firstinspires.ftc.teamcode.constants.ReferenceHolder
import org.firstinspires.ftc.teamcode.constants.ReferenceHolder.Companion.setRobotRef
import org.firstinspires.ftc.teamcode.utilities.addData

abstract class MOEOpMode : LinearOpMode(), MOEFirebase {
    val firelog = MOETelemetry(telemetry)
    lateinit var ref: DatabaseReference
    lateinit var robot: MOEBot
    lateinit var mainGamepad: MOEGamePad
    final override fun runOpMode() {
        moeDoubleInternalInit()
        moeInternalInit()
        setRobotRef(robot)
        initOpMode()
        notifyTelemetry()
        waitForStart()
        run()
    }

    private fun moeDoubleInternalInit() {
        setRefs()
        createGamePads()
        addListener()
    }

    private fun createGamePads() {
        mainGamepad = MOEGamePad(gamepad1)
    }


    private fun setRefs() {
        ReferenceHolder.setRefs(this)
    }

    private fun addListener() {
        val customRef = getCustomRef(MOEConfig.config) ?: return
        customRef.addValueEventListener(object : MOEEventListener() {
            override fun onDataChange(snapshot: DataSnapshot) {
                onConfigChanged(snapshot)
            }
        })
        ref = customRef

    }

    private fun notifyTelemetry() {
        telemetry.addData("waiting for init")
        telemetry.update()

    }

    abstract fun moeInternalInit()


    abstract fun initOpMode()


    abstract fun run()

}