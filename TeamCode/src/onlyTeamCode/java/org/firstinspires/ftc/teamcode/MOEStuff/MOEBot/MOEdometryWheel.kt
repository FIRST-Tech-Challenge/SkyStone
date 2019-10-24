package org.firstinspires.ftc.teamcode.MOEStuff.MOEBot

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.OdometryWheelConfig.*
import org.firstinspires.ftc.teamcode.constants.MOEConstants
import org.firstinspires.ftc.teamcode.constants.MOEConstants.Odometry.Wheels
import org.firstinspires.ftc.teamcode.constants.ReferenceHolder.Companion.hardwareMap
import org.firstinspires.ftc.teamcode.utilities.closestAngleDifference

const val MIN_VOLTAGE: Double = 0.0
const val MAX_VOLTAGE: Double = 5.0

data class OdometryWheelConfig(val name: String, val direction: Direction, val orientation: Orientation) {
    enum class Direction { FORWARD, REVERSE }

    enum class Orientation {
        AXIAL, STRAFE;

        private fun circumference(): Double {
            return when (this) {
                AXIAL -> Wheels.Circumference.AXIAL
                STRAFE -> Wheels.Circumference.AXIAL
            }
        }

        fun rotationalScalar(): Double = circumference() / (2 * Math.PI)
    }
}

class MOEdometryWheel(config: OdometryWheelConfig) {


    val odometry: AnalogInput = hardwareMap.get(AnalogInput::class.java, config.name)

    private val direction: Direction = config.direction
    private val directionScalar: Double = if (direction == Direction.FORWARD) 1.0 else -1.0
    private val orientation: Orientation = config.orientation;
    private val rotationScalar: Double = orientation.rotationalScalar()

    var odometryValue: Double = 0.0
    private var prevVoltage: Double = 0.0
    private var totalVoltage: Double = 0.0
    var odometryChange: Double = 0.0
    private var prevAngle: Double = 0.0
    private val offsetAdd: Double = 0.0

//    fun getAngleOdometryOffset(): Double = 1.0

    fun getVoltage() = odometry.voltage * directionScalar

    init {
        prevVoltage = getVoltage()
    }

    fun updateOdometry() {
        val currentVoltage = getVoltage()


        val voltageChange = if (prevVoltage < currentVoltage) {
            val innerRange = currentVoltage - prevVoltage
            val outerRange = MAX_VOLTAGE - innerRange
            if (innerRange < outerRange) innerRange else -outerRange
        } else {
            val innerRange = prevVoltage - currentVoltage
            val outerRange = MAX_VOLTAGE - innerRange
            if (innerRange < outerRange) -innerRange else outerRange
        }

        totalVoltage += voltageChange
        odometryValue = totalVoltage

        prevVoltage = currentVoltage
    }
}
