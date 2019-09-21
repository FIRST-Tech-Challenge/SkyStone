package example

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

// Tank drive with a two motor drivetrain
@Disabled
@TeleOp(name = "Two Motor Tank Drive")
class TankDrive2 : OpMode() {
    lateinit var drivetrain : Drivetrain
    override fun init() {
        drivetrain = Drivetrain2(hardwareMap)
    }

    override fun loop() {
        drivetrain.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y)
    }
}

// Tank drive with a four motor drive train
@Disabled
@TeleOp(name = "Four Motor Tank Drive")
class TankDrive4 : OpMode() {
    lateinit var drivetrain : Drivetrain
    override fun init() {
        drivetrain = Drivetrain4(hardwareMap)
    }

    override fun loop() {
        drivetrain.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y)
    }
}

// Single stick tank drive
@Disabled
@TeleOp(name = "Single Stick Tank")
class SingleStickTank : OpMode() {
    lateinit var drivetrain : Drivetrain4

    override fun init() {
        drivetrain = Drivetrain4(hardwareMap)
    }

    override fun loop() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = gamepad1.left_stick_y.toDouble()
        // @todo: How can you do tank drive with only the left joystick?
    }
}


// Basic mecanum teleop. Requires a four motor drive train
@Disabled
@TeleOp(name = "Mecanum")
class Mecanum : OpMode() {
    lateinit var drivetrain : Drivetrain4

    override fun init() {
        drivetrain = Drivetrain4(hardwareMap)
    }

    override fun loop() {
        drivetrain.holonomicDriveSticks(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)
    }
}

// A quick example of using sensors in teleop:
// use the IMU such that the robot, no matter its orientation, moves in the driver's
// frame of reference as opposed to the robot's frame of reference.
// @todo: Test. This may or may not work :)
@Disabled
@TeleOp(name = "Arcade Mecanum")
class ArcadeMecanum : OpMode() {
    lateinit var drivetrain : Drivetrain4
    lateinit var imu : BNO055IMU
    var referenceGyroPosition = 0.0

    override fun init() {
        drivetrain = Drivetrain4(hardwareMap)
        val params = BNO055IMU.Parameters()
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        params.loggingEnabled = false
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(params)
        resetGyro()
    }

    private fun getOrientation() : Double {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS)
                .firstAngle.toDouble()
    }

    private fun resetGyro() {
        referenceGyroPosition = getOrientation()
    }

    override fun loop() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = gamepad1.left_stick_y.toDouble()
        val rotation = gamepad1.right_stick_x.toDouble()
        val currentOrientation = getOrientation()

        val direction = Math.atan2(y, x) - (currentOrientation - referenceGyroPosition)
        val speed = Math.sqrt(x * x + y * y)

        drivetrain.holonomicDrive(direction, speed, rotation)

        if (gamepad1.a) {
            resetGyro()
        }

        telemetry.addData("Orientation", Math.toDegrees(currentOrientation))
        telemetry.addData("Reference", Math.toDegrees(referenceGyroPosition))
        telemetry.addData("Difference", Math.toDegrees(currentOrientation - referenceGyroPosition))
        telemetry.update()
    }
}