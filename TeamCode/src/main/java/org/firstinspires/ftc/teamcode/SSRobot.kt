package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.*

/**
 * Created by KasaiYuki on 9/25/2018.
 */
class SSRobot {

    var hwdMap: HardwareMap? = null
    var leftDrive: DcMotor? = null
    var rightDrive: DcMotor? = null
    var linSlideY: DcMotor? = null


    var motF = DcMotorSimple.Direction.FORWARD
    var motR = DcMotorSimple.Direction.REVERSE
    var serR = Servo.Direction.REVERSE
    var serF = Servo.Direction.FORWARD

    //Default Constructor
    init {

    }

    fun init(ahwdMap: HardwareMap) {
        //hardware maping motors, servos, and sensors
        hwdMap = ahwdMap
        leftDrive = ahwdMap.dcMotor.get("leftDrive")
        rightDrive = ahwdMap.dcMotor.get("rightDrive")
        linSlideY = ahwdMap.dcMotor.get("linSlideY")


        //Setting direction
        leftDrive?.direction = motF
        rightDrive?.direction = motR
        linSlideY?.direction = motR


        leftDrive?.power = 0.0
        rightDrive?.power = 0.0
        leftDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

    }
    fun init(ahwdMap: HardwareMap, diff: Boolean)
    {
        //hardware mapping motors, servos, and sensors
        hwdMap = ahwdMap
        leftDrive = ahwdMap.dcMotor.get("leftDrive")
        rightDrive = ahwdMap.dcMotor.get("rightDrive")
        linSlideY = ahwdMap.dcMotor.get("linSlideY")


        //Setting direction
        leftDrive?.direction = motF
        rightDrive?.direction = motR
        linSlideY?.direction = motR


        leftDrive?.power = 0.0
        rightDrive?.power = 0.0
        leftDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        linSlideY?.mode = DcMotor.RunMode.RUN_USING_ENCODER

    }

    //METHODS

    fun drive(leftM: Double, rightM: Double) {
        leftDrive?.power = leftM
        rightDrive?.power = rightM
    }

    fun drive(pow: Double)//OVERLOAD-both motors run at same velocity
    {
        drive(pow, pow)
    }

    fun brake() {
        this.drive(0.0)
    }

    fun liftSlideY(pow: Float) {
        linSlideY?.power = pow.toDouble()
    }
}

