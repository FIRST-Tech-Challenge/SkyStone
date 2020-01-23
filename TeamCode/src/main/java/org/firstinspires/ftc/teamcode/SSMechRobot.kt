package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*

/*
    TODO: Create interfaces that lead to abstract class and seperate auto and tele
 */


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
    var rightHook: Servo? = null
    var leftHook: Servo? = null
    var touch: DigitalChannel? = null


    var motF = DcMotorSimple.Direction.FORWARD
    var motR = DcMotorSimple.Direction.REVERSE
    var serR = Servo.Direction.REVERSE
    var serF = Servo.Direction.FORWARD

    fun init(ahwdMap: HardwareMap) {
        //hardware maping motors, servos, and sensors
        //TODO: use list and iterate over
        var dcList = mutableListOf<DcMotor>() //creating empty list for dc motor
        //increment for each var above
        //map for each dc
        //similar for servo and sensor
        hwdMap = ahwdMap
        var motorList = arrayListOf<DcMotor?>(bLDrive, bRDrive, fLDrive, fRDrive, vSlide)
        for (i in 0..motorList.size) {
            motorList[i] = ahwdMap.dcMotor.get(motorList[i].toString())
        }

        var servoList = arrayListOf<Servo?>(hSlide, claw, leftHook, rightHook)
        (0..servoList.size).forEach { i -> servoList[i] = ahwdMap.servo.get(servoList[i].toString())}

/*        bLDrive = ahwdMap.dcMotor.get("bLDrive")
        bRDrive = ahwdMap.dcMotor.get("bRDrive")
        fLDrive = ahwdMap.dcMotor.get("fLDrive")
        fRDrive = ahwdMap.dcMotor.get("fRDrive")
        vSlide = ahwdMap.dcMotor.get("vSlide")
        hSlide = ahwdMap.servo.get("hSlide")
        claw = ahwdMap.servo.get("claw")
        leftHook = ahwdMap.servo.get("leftHook")
        rightHook = ahwdMap.servo.get("rightHook")*/
        touch = ahwdMap.digitalChannel.get("touch")

        //Setting direction
        bLDrive?.direction = motF
        bRDrive?.direction = motR
        fLDrive?.direction = motF
        fRDrive?.direction = motR
        vSlide?.direction = motR
        hSlide?.direction = serR
        claw?.direction = serF
        rightHook?.direction = serR
        leftHook?.direction = serF


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
        bLDrive?.power = -pow
        fLDrive?.power = -pow
    }

    fun rightPow(pow: Double){
        bRDrive?.power = -pow
        fRDrive?.power = -pow
    }

    fun strafe(pow: Double) //Positive Value = Left Strafe || Negative Value = Right Strafe
    {
        bLDrive?.power = -pow
        fLDrive?.power = pow
        bRDrive?.power = pow
        fRDrive?.power = -pow
    }

    fun drive(leftM: Double, rightM: Double) { //used for turning
        leftPow(leftM)
        rightPow(rightM * 1.5)
    }

    fun drive(pow: Double)//OVERLOAD-both motors run at same velocity
    {
        drive(pow, pow)
    }


    fun brake() {
        this.drive(0.0)
    }

    fun dropHook(gp: Gamepad)
    {
/*        var down = false
        var changed = false

        if(gp.a and !changed) {
            if(!gp.a)  down = !down
            changed = true
        } else if(!gp.a) changed = false

        if(down) { //up
            this.leftHook?.position = 0.0
            this.rightHook?.position = 0.0
        }
        else { //down
            this.leftHook?.position = 0.7
            this.rightHook?.position = 0.7
        }*/
        if(gp.a) { //hook down
            this.leftHook?.position = 0.7
            this.rightHook?.position = 0.72
        }
        else { //default position
            this.leftHook?.position = 0.18
            this.rightHook?.position = 0.21
        }

    }



    fun pinch(gp: Gamepad) {
        if(gp.left_bumper) { //hook down
            this.claw?.position = 0.00
        }
        else { //default position
            this.claw?.position = 0.28
        }
        /*
        Toggle Function
         */
/*        var pushedBefore = false
        if(gp.left_bumper)
        {
            if (pushedBefore)
            {
                this.claw?.position = 0.28
                pushedBefore = false
            }
            else if(!pushedBefore)
            {
                this.claw?.position = 0.00
                pushedBefore = true
            }
        }*/
    }
}

