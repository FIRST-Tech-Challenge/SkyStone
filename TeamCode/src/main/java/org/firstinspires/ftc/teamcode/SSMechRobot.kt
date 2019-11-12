package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*

/**
 * Created by KasaiYuki on 9/25/2018.
 */
class SSMechRobot {

    var hwdMap: HardwareMap? = null
    var bLDrive: DcMotor? = null
    var bRDrive: DcMotor? = null
    var fLDrive: DcMotor? = null
    var fRDrive: DcMotor? = null
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
        bLDrive = ahwdMap.dcMotor.get("bLDrive")
        bRDrive = ahwdMap.dcMotor.get("bRDrive")
        fLDrive = ahwdMap.dcMotor.get("fLDrive")
        fRDrive = ahwdMap.dcMotor.get("fRDrive")
        vSlide = ahwdMap.dcMotor.get("vSlide")
        hSlide = ahwdMap.servo.get("hSlide")
        claw = ahwdMap.servo.get("claw")
        touch = ahwdMap.digitalChannel.get("touch")

        //Setting direction
        bLDrive?.direction = motF
        bRDrive?.direction = motR
        fLDrive?.direction = motF
        fRDrive?.direction = motR
        vSlide?.direction = motR
        hSlide?.direction = serR
        claw?.direction = serF


        bLDrive?.power = 0.0
        bRDrive?.power = 0.0
        fLDrive?.power = 0.0
        fRDrive?.power = 0.0
        bLDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        bRDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //Use encoders for linear slide motor

    }

    //METHODS

    fun leftPow(pow: Double){
        bLDrive?.power = pow
        fLDrive?.power = pow
    }

    fun rightPow(pow: Double){
        bRDrive?.power = pow
        fRDrive?.power = pow
    }

    fun strafe(pow: Double)
    {
        bLDrive?.power = pow
        fLDrive?.power = -pow
        bRDrive?.power = pow
        fRDrive?.power = -pow
    }

    fun drive(leftM: Double, rightM: Double) {
        leftPow(leftM)
        rightPow(rightM)
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
        var changed = false
        if(gp.left_bumper and !changed) {
            if(!gp.left_bumper) close = !close
            changed = true
        } else if(!gp.left_bumper) changed = false
        when {
            close -> this.claw?.position = 0.0 //close
            !close -> this.claw?.position = 1.0 //open-default
        }
    }
}

