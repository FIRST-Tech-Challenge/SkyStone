package org.firstinspires.ftc.teamcode.MOEStuff.MOEBot

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utilities.lerp
import org.firstinspires.ftc.teamcode.constants.ReferenceHolder.Companion.hardwareMap
import kotlinx.coroutines.*

data class ServoConfig(val name: String, val min: Double, val max: Double, val direction: Servo.Direction)
class MOEServo(config: ServoConfig) {
    private var mServo: Servo = hardwareMap.get(Servo::class.java, config.name)

    init {
        setRange(config.min, config.max)
        setDirection(config.direction)
    }

    private fun getPosition(): Double = mServo.position

    fun setPosition(position: Double) {
        mServo.position = position;
    }

    fun setPositionOverTime(destPosition: Double, duration: Double, async: Boolean) {
        val initialPosition = getPosition()
        val launch = GlobalScope.launch {
            val time = ElapsedTime()
            val range = initialPosition..destPosition
            while (time.seconds() < duration) {
                setPosition(range.lerp(time.seconds() / duration))
            }
        }

        if (!async) {
            runBlocking { launch.join() }
        }
    }

    fun setRange(min: Double, max: Double) {
        mServo.scaleRange(min, max)
    }

    fun setDirection(direction: Servo.Direction) {
        mServo.direction = direction;
    }
}
