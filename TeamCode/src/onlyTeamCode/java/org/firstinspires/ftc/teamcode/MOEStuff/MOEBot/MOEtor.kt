package org.firstinspires.ftc.teamcode.MOEStuff.MOEBot

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.teamcode.constants.ReferenceHolder.Companion.moeOpMode

data class MotorConfig(val name: String, val direction: Direction = Direction.FORWARD,
                       val zeroPowerBehavior: ZeroPowerBehavior = BRAKE)


class MOEtor(config: MotorConfig) {
    private var mMotor: DcMotorEx = moeOpMode.hardwareMap.get(
            DcMotorEx::class.java, config.name)

    init {

        setDirection(config.direction)
        setZeroPowerBehavior(config.zeroPowerBehavior)
    }

    fun setPower(power: Double) {
        mMotor.power = power;
    }

    fun setDirection(direction: Direction) {
        mMotor.direction = direction;
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior) {
        mMotor.zeroPowerBehavior = zeroPowerBehavior;
    }
}
