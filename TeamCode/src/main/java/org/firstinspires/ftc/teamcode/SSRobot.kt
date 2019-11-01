package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*

/**
 * Created by KasaiYuki on 9/25/2018.
 */
class SSRobot {

    var hwdMap: HardwareMap? = null
    var leftDrive: DcMotor? = null
    var rightDrive: DcMotor? = null
    var vSlide: DcMotor? = null
    var hSlide: Servo? = null
    var claw: Servo? = null
    var touch: DigitalChannel? = null


    var motF = DcMotorSimple.Direction.FORWARD
    var motR = DcMotorSimple.Direction.REVERSE
    var serR = Servo.Direction.REVERSE
    var serF = Servo.Direction.FORWARD

    fun init(ahwdMap: HardwareMap) {
        //hardware maping motors, servos, and sensors
        hwdMap = ahwdMap
        leftDrive = ahwdMap.dcMotor.get("leftDrive")
        rightDrive = ahwdMap.dcMotor.get("rightDrive")
        vSlide = ahwdMap.dcMotor.get("vSlide")
        hSlide = ahwdMap.servo.get("hSlide")
        claw = ahwdMap.servo.get("claw")
        touch = ahwdMap.digitalChannel.get("touch")

        //Setting direction
        leftDrive?.direction = motF
        rightDrive?.direction = motR
        vSlide?.direction = motR
        hSlide?.direction = serR
        claw?.direction = serF


        leftDrive?.power = 0.0
        rightDrive?.power = 0.0
        leftDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //Use encoders for linear slide motor

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


    fun pinch(gp: Gamepad) {
        var close = false
        if(gp.left_bumper) close = !close
        when {
            close -> this.claw?.position = 0.0 //close
            !close -> this.claw?.position = 1.0 //open-default
        }
    }
}

