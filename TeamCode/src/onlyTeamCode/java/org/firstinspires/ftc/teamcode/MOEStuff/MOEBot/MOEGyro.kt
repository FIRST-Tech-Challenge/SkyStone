package org.firstinspires.ftc.teamcode.subsystems.imu

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.util.ReadWriteFile
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.constants.ReferenceHolder
import org.firstinspires.ftc.teamcode.constants.ReferenceHolder.Companion.hardwareMap

import java.io.File

class MOEGyro {
    private val imu = hardwareMap.get(BNO055IMU::class.java, "imu")

    private var offset: Double = 0.toDouble()
    //TODO: check angles

    val angle: Double
        get() = -imu.angularOrientation.firstAngle - offset


    /**
     * Initializes IMU parameters
     */
    fun initialize() {
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = AngleUnit.DEGREES
        imu.initialize(parameters)

    }

    /**
     * Sets an offset for the IMU angle values
     * @param offset the offset to set
     */
    fun setOffset(offset: Double) {
        this.offset = offset
    }

    /**
     * Set the current angle as 0
     */
    fun setAsZero() {
        offset = (-imu.angularOrientation.firstAngle).toDouble()
    }

}