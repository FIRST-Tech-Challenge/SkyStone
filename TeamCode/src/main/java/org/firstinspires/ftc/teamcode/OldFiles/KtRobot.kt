package org.firstinspires.ftc.teamcode.OldFiles

import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.*

/**
 * Created by KasaiYuki on 9/25/2018.
 */
class KtRobot
{
    val swingUp = 0.15
    val swingDown = 0.4

    var hwdMap: HardwareMap? = null
    var leftDrive: DcMotor? = null
    var rightDrive: DcMotor? = null
    var flag: Servo? = null //servo to drop team token
    var lSlideArm: DcMotor? = null //motor to lift the linear slide
    var touch: DigitalChannel? = null
    var stick: CRServo? = null

    var motF = DcMotorSimple.Direction.FORWARD
    var motR = DcMotorSimple.Direction.REVERSE
    var serR = Servo.Direction.REVERSE
    var serF = Servo.Direction.FORWARD

    //Default Constructor
    init
    {

    }

    fun init(ahwdMap: HardwareMap)
    {
        hwdMap = ahwdMap
        leftDrive = ahwdMap.dcMotor.get("bLDrive")
        rightDrive = ahwdMap.dcMotor.get("bRDrive")
        flag = ahwdMap.servo.get("flag")
        lSlideArm = ahwdMap.dcMotor.get("lSlideArm")
        touch = ahwdMap.digitalChannel.get("touch")
        stick = ahwdMap.crservo.get("stick")

        //Setting direction
        leftDrive?.direction = motF
        rightDrive?.direction = motR
        flag?.direction = serR //0-up, CCW
        stick?.direction = motF

        leftDrive?.power = 0.0
        rightDrive?.power = 0.0
        leftDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightDrive?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }



    //METHODS

    fun swingStick(gp: Gamepad)
    {
        //var change = true

        when {
            gp.x -> stick?.power = 0.55
            gp.y -> stick?.power = -0.45
            else -> stick?.power = 0.0
        }
        Thread.sleep(250)
        stick?.power = 0.5 //stationary
    }
    fun rightMove(pow: Double) //runs right motor
    {
        this.rightDrive?.power = pow
    }
    fun leftMove(pow: Double) //runs left motor
    {
        this.leftDrive?.power = pow
    }

    fun swing(pos: Double) //swings flag to given positiom
    {
        this.flag?.position = pos
    }


    fun drive(leftM: Double, rightM: Double)
    {
        leftDrive?.power = leftM
        rightDrive?.power = rightM
    }

    fun drive(pow: Double)//OVERLOAD
    {
        drive(pow, pow)
    }

    fun brake() {
        this.drive(0.0)
    }

    fun swingFlag(gp: Gamepad) {
        var change = true
        when {
            gp.right_bumper -> change = !change
        }
        when {
            change -> flag?.position = swingUp //UP
            !change -> flag?.position = swingDown //DOWN
        }
    }

    fun liftRobot(pow: Double)
    {
        lSlideArm?.power = pow
    }

    /* GETTING MINERALS */

    fun getCube(dir: String)
    {
        val f1: Long = 250
        val rightTurn: Long = 200
        val dist: Long = 50
        val rightTurnPow = 0.5
        val quartPow = 0.25
        val quart: Long = 150
        val dist2: Long = 27
        //val center: Boolean = false;

        if(dir == "left")
        {
            this.rightDrive?.power = rightTurnPow
            Thread.sleep(rightTurn)//90 degree turn
            this.drive(0.5)
            Thread.sleep(dist)
            this.leftDrive?.power = rightTurnPow
            Thread.sleep(rightTurn)//90 degree
            this.drive(0.5)
            Thread.sleep(f1)
            //OPEN GATE AND CAPTURE CUBE
            this.leftDrive?.power = quartPow//go to depot
            Thread.sleep(quart)
            this.drive(0.5)
            Thread.sleep(dist2)
            this.drive(-1.0)
            Thread.sleep(50)
            this.drive(0.25)
            Thread.sleep(500)
            this.drive(-1.0)
            Thread.sleep(200)
        }
        else if(dir == "right")
        {
            this.leftDrive?.power = rightTurnPow
            Thread.sleep(rightTurn)//90 degree turn
            this.drive(0.5)
            Thread.sleep(dist)
            this.rightDrive?.power = rightTurnPow
            Thread.sleep(rightTurn)//90 degree
            this.drive(0.5)
            Thread.sleep(f1)
            //OPEN GATE AND CAPTURE CUBE
            this.rightDrive?.power = quartPow//go to depot
            Thread.sleep(quart)
            this.drive(0.5)
            Thread.sleep(dist2)
            this.drive(-1.0)
            Thread.sleep(50)
            this.drive(0.25)
            Thread.sleep(500)
            this.drive(-1.0)
            Thread.sleep(200)
            this.drive(0.0)
        }

        else if(dir == "center")
        {
            this.flag?.position = 0.38
            this.drive(0.5)
            Thread.sleep(6000)//6000 for full field
            this.drive(-1.0)
            Thread.sleep(500)
            this.drive(0.5)
            Thread.sleep(500)
            this.drive(0.0)

        }

        else
        {
            this.drive(0.5)
            Thread.sleep(5000)
            this.drive(-1.0)
            Thread.sleep(500)
            this.drive(0.25)
            Thread.sleep(500)
            this.drive(-1.0)
            Thread.sleep(2000)
            this.drive(0.0)
        }




    }

    fun drop()
    {
        //robot.liftRobot(1.0)//dropping down
        while(touch!!.state)
            liftRobot(1.0)//dropping down
        //sleep(dropTime)
        liftRobot(0.0)

        //wiggle out
        leftDrive?.power = 0.35
        rightDrive?.power = -0.25
        Thread.sleep(250)
        brake()

        Thread.sleep(1000)
        drive(0.25)
        Thread.sleep(250)
        leftDrive?.power = -0.315 //re align
        rightDrive?.power = 0.25
        Thread.sleep(250)
        brake()
    }
}