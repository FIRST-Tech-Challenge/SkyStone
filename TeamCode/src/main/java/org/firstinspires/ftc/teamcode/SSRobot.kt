package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*

/**
 * Created by KasaiYuki on 9/25/2018.
 */
class SSRobot {

    var hwdMap: HardwareMap? = null
    var leftDrive: DcMotor? = null
    var rightDrive: DcMotor? = null
    var linSlideY: DcMotor? = null
    var linSlideX: CRServo? = null
    var claw: Servo? = null


    var motF = DcMotorSimple.Direction.FORWARD
    var motR = DcMotorSimple.Direction.REVERSE
    var serR = Servo.Direction.REVERSE
    var serF = Servo.Direction.FORWARD

    fun init(ahwdMap: HardwareMap) {
        //hardware maping motors, servos, and sensors
        hwdMap = ahwdMap
        leftDrive = ahwdMap.dcMotor.get("leftDrive")
        rightDrive = ahwdMap.dcMotor.get("rightDrive")
        linSlideY = ahwdMap.dcMotor.get("linSlideY")
        linSlideX = ahwdMap.crservo.get("linSlideX")
        claw = ahwdMap.servo.get("claw")


        //Setting direction
        leftDrive?.direction = motF
        rightDrive?.direction = motR
        linSlideY?.direction = motR
        linSlideX?.direction = motF
        claw?.direction = serF


        leftDrive?.power = 0.0
        rightDrive?.power = 0.0
        leftDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun init(ahwdMap: HardwareMap, encoder: Boolean) = if (encoder) { //used when enabling encoders on motors
            //hardware mapping motors, servos, and sensors
            hwdMap = ahwdMap
            leftDrive = ahwdMap.dcMotor.get("leftDrive")
            rightDrive = ahwdMap.dcMotor.get("rightDrive")
            linSlideY = ahwdMap.dcMotor.get("linSlideY")
            claw = ahwdMap.servo.get("claw")


            //Setting direction
            leftDrive?.direction = motF
            rightDrive?.direction = motR
            linSlideY?.direction = motR
            claw?.direction = serF


            leftDrive?.power = 0.0
            rightDrive?.power = 0.0
            leftDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            rightDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            linSlideY?.mode = DcMotor.RunMode.RUN_USING_ENCODER //Use encoders for linear slide motor
        } else this.init(ahwdMap)

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


    fun pinch(gp: Gamepad) {
        this.claw?.position = when {
            gp.a -> 0.1
            else -> 0.25 //can be explicit
        }
    }
}

