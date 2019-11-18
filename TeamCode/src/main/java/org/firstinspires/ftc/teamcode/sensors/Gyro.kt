package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.util.Factory

/**
 * Public interface representing the set of methods every single gyro sensor is guaranteed to implement.
 * This makes it useful for creating op modes that work for any gyro sensors other than the IMUGyro.
 *
 * For the uninitiated, a gyro sensor measures the orientation of a robot. That's it. That's all it does.
 *
 * You do not construct instances of Gyro directly; instead, you use a subclass such as IMUGyro.
 */
interface Gyro {
    /**
     * Initializes the gyro. Usually called in an op mode's init phase.
     * Implementation details vary by specific gyro sensor.
     */
    fun initialize()

    /**
     * Calibrates the gyro, or begins the calibration process. Usually called immediately after initialize.
     * Implementation details vary by specific gyro sensor.
     * In some cases, you may have to wait a few seconds before the gyro is calibrated. See isCalibrated.
     */
    fun calibrate()

    /**
     * Whether the gyro has been successfully calibrated.
     * See GyroTestOpMode for a demonstration of how to wait for a gyro to calibrate.
     */
    val isCalibrated: Boolean

    /**
     * Returns the angle that the robot is facing.
     *
     * Negative numbers indicate that the robot is facing left from where it originally started.
     * Likewise, positive numbers indicate that the robot is facing right.
     *
     * @return the angle that the robot is facing
     */
    val angle: Double
}

/**
 * A Gyro instance that uses the REV Robotic Expansion Hub's built-in gyro sensor (IMU) to obtain measurements.
 */
class BNO055IMUGyro(val imu: BNO055IMU, val parameters: BNO055IMU.Parameters): Gyro {
    override fun initialize() {
        imu.initialize(parameters)
    }

    override fun calibrate() {}
    override val isCalibrated get() = imu.isGyroCalibrated

    /**
     * A number which all gyro measurements are multiplied by. Used to ensure that getAbsoluteAngle() conforms to the spec laid out in Gyro.
     * You don't need to know how this works to use IMUGyro.
     */
    val angleModifier = -1.0

    override val angle =
            imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble() * angleModifier

    companion object: Factory<BNO055IMUGyro> {
        val IMU_NAME = "imu"
        val IMU_PARAMETERS: BNO055IMU.Parameters get() {
            val parameters = BNO055IMU.Parameters()
            parameters.mode = BNO055IMU.SensorMode.IMU
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            parameters.loggingEnabled = false
            return parameters
        }

        /**
         * @inheritDoc
         */
        @JvmStatic override fun standard(hardwareMap: HardwareMap): BNO055IMUGyro {
            return BNO055IMUGyro(hardwareMap[BNO055IMU::class.java, IMU_NAME], IMU_PARAMETERS)
        }
    }
}