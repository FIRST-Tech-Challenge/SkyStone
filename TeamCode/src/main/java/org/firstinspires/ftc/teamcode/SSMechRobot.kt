package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*

/**
 * Created by KasaiYuki on 9/25/2018.
 */
class SSMechRobot {

    var hwdMap: HardwareMap? = null
    var lBDrive: DcMotor? = null
    var rBDrive: DcMotor? = null
    var lFDrive: DcMotor? = null
    var rFDrive: DcMotor? = null
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
        lBDrive = ahwdMap.dcMotor.get("lBDrive")
        rBDrive = ahwdMap.dcMotor.get("rBDrive")
        lFDrive = ahwdMap.dcMotor.get("lFDrive")
        rFDrive = ahwdMap.dcMotor.get("rFDrive")
        vSlide = ahwdMap.dcMotor.get("vSlide")
        hSlide = ahwdMap.servo.get("hSlide")
        claw = ahwdMap.servo.get("claw")
        touch = ahwdMap.digitalChannel.get("touch")

        //Setting direction
        lBDrive?.direction = motF
        rBDrive?.direction = motR
        lFDrive?.direction = motF
        rFDrive?.direction = motR
        vSlide?.direction = motR
        hSlide?.direction = serR
        claw?.direction = serF


        lBDrive?.power = 0.0
        rBDrive?.power = 0.0
        lFDrive?.power = 0.0
        rFDrive?.power = 0.0
        lBDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rBDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //Use encoders for linear slide motor

    }

    //METHODS

    fun leftPow(pow: Double){
        lBDrive?.power = pow
        lFDrive?.power = pow
    }

    fun rightPow(pow: Double){
        rBDrive?.power = pow
        rFDrive?.power = pow
    }

    fun strafe(pow: Double)
    {
        lBDrive?.power = pow
        lFDrive?.power = -pow
        rBDrive?.power = pow
        rFDrive?.power = -pow
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

