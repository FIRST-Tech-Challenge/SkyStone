package example

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class EncoderVals2(left : Int, right : Int)
class EncoderVals4(lf : Int, lb : Int, rf : Int, rb : Int)

abstract class Drivetrain {
    abstract fun drive(leftPower : Double, rightPower : Double)
    abstract fun setMotorMode(mode : DcMotor.RunMode)
    abstract fun setEncoderTargets(left : Int, right : Int)
    abstract fun getEncoderValues2() : EncoderVals2
    abstract fun allMotorsBusy() : Boolean
    abstract fun anyMotorsBusy() : Boolean

    fun tankDrive(leftStickY : Double, rightStickY : Double) {
        drive(leftStickY, rightStickY)
    }

    fun tankDrive(leftStickY : Float, rightStickY : Float) {
        drive(leftStickY.toDouble(), rightStickY.toDouble())
    }
}

class Drivetrain4(hwm : HardwareMap) : Drivetrain() {
    val lf = hwm.dcMotor.get("lf")
    val lb = hwm.dcMotor.get("lb")
    val rf = hwm.dcMotor.get("rf")
    val rb = hwm.dcMotor.get("rb")

    init {
        rf.direction = DcMotorSimple.Direction.REVERSE
        rb.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun drive(leftPower: Double, rightPower: Double) {
        lf.power = leftPower
        lb.power = leftPower
        rf.power = rightPower
        rb.power = rightPower
    }

    override fun setMotorMode(mode: DcMotor.RunMode) {
        lf.mode = mode
        lb.mode = mode
        rf.mode = mode
        rb.mode = mode
    }

    override fun setEncoderTargets(left: Int, right: Int) {
        lf.targetPosition = left
        lb.targetPosition = left
        rf.targetPosition = right
        rb.targetPosition = right
    }

    fun setEncoderTargets(lft: Int, lbt: Int, rft: Int, rbt: Int) {
        lf.targetPosition = lft
        lb.targetPosition = lbt
        rf.targetPosition = rft
        rb.targetPosition = rbt
    }

    override fun allMotorsBusy(): Boolean {
        return lf.isBusy && lb.isBusy && rf.isBusy && rb.isBusy
    }

    override fun anyMotorsBusy(): Boolean {
        return lf.isBusy || lb.isBusy || rf.isBusy || rb.isBusy
    }

    override fun getEncoderValues2(): EncoderVals2 {
        return EncoderVals2((lf.currentPosition + lb.currentPosition) / 2,
                (rf.currentPosition + rb.currentPosition) / 2)
    }

    fun getEncoderVals4(): EncoderVals4 {
        return EncoderVals4(lf.currentPosition, lb.currentPosition, rf.currentPosition, rb.currentPosition)
    }

    fun drive(leftFrontPower: Double, leftBackPower: Double, rightFrontPower: Double, rightBackPower: Double) {
        lf.power = leftFrontPower
        lb.power = leftBackPower
        rf.power = rightFrontPower
        rb.power = rightBackPower
    }

    fun holonomicDrive(direction: Double, speed: Double, rotation: Double) {
        val scale = Math.max(1.0, speed + Math.abs(rotation))
        val sin = Math.sin(direction + Math.PI / 4.0)
        val cos = Math.cos(direction + Math.PI / 4.0)

        drive((sin + rotation) / scale,
                (cos + rotation) / scale,
                (cos - rotation) / scale,
                (sin - rotation) / scale)
    }

    fun holonomicDriveSticks(leftStickX: Double, leftStickY: Double, rightStickX: Double) {
        holonomicDrive(Math.atan2(leftStickY, leftStickX),
                Math.sqrt(Math.pow(leftStickX, 2.0) + Math.pow(leftStickY, 2.0)),
                rightStickX)
    }

    fun holonomicDriveSticks(leftStickX: Float, leftStickY: Float, rightStickX: Float) {
        holonomicDriveSticks(leftStickX.toDouble(), leftStickY.toDouble(), rightStickX.toDouble())
    }
}

class Drivetrain2(hwm : HardwareMap) : Drivetrain() {
    val leftMotor = hwm.dcMotor.get("leftMotor")
    val rightMotor = hwm.dcMotor.get("rightMotor")

    init {
        rightMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun drive(leftPower: Double, rightPower: Double) {
        leftMotor.power = leftPower
        rightMotor.power = rightPower
    }

    override fun setMotorMode(mode: DcMotor.RunMode) {
        leftMotor.mode = mode
        rightMotor.mode = mode
    }

    override fun setEncoderTargets(left: Int, right: Int) {
        leftMotor.targetPosition = left
        rightMotor.targetPosition = right
    }

    override fun getEncoderValues2(): EncoderVals2 {
        return EncoderVals2(leftMotor.currentPosition, rightMotor.currentPosition)
    }

    override fun allMotorsBusy(): Boolean {
        return leftMotor.isBusy && rightMotor.isBusy
    }

    override fun anyMotorsBusy(): Boolean {
        return leftMotor.isBusy || rightMotor.isBusy
    }
}