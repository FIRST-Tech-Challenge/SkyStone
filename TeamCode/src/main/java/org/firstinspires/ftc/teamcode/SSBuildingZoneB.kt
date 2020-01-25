package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor


/**
 * Created by KasaiYuki on 9/21/2018.
 */
@Autonomous(name = "SSBuildingZoneB", group = "Autonomous")
//@Disabled
class SSBuildingZoneB : LinearOpMode()
{
    val robot = SSMechRobot()

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        telemetry.addData("Status: ", "Autonomous Initialized")
        telemetry.update()

        robot.init(hardwareMap)
        //robot.vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        //robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.vSlide?.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.vSlide?.targetPosition = robot.vSlide!!.currentPosition

        waitForStart()
        robot.leftHook?.position = 0.0
        robot.rightHook?.position = 0.0
        //robot.vSlide?.targetPosition = 50 + robot.vSlide!!.currentPosition
        robot.vSlide?.power = 1.0
        pause()
        robot.drive(0.50) //Drives Forward to the Stones
        sleep(1700)
        pause()
        robot.claw?.position = 0.0
        pause()
        sleep(500)
        /*robot.vSlide?.targetPosition = 50 + robot.vSlide!!.currentPosition
        sleep(500)*/
        robot.drive(-0.50)
        sleep(750)
        pause()
        robot.strafe(1.0)//Heads to Foundation
        sleep(3550)
        pause()
        robot.vSlide?.targetPosition = 2500 + robot.vSlide!!.currentPosition
        sleep(750)
        robot.hSlide?.position = 0.3
        sleep(550)
        robot.hSlide?.position = 0.5
        robot.drive(0.5)
        sleep(900)
        pause()
        robot.claw?.position = robot.clawPinchPos
        pause()
        robot.drive(-0.5)
        sleep(1200)
        pause()
        robot.vSlide?.targetPosition = robot.vSlide!!.currentPosition - 2500
        sleep(750)
        pause()
        robot.hSlide?.position = 0.7
        sleep(550)
        pause()
        robot.strafe(-0.5)
        sleep(2100)
        robot.claw?.position = 0.0
        pause()
        robot.strafe(-0.5)
        sleep(2400)
        pause()
        //robot.vSlide?.targetPosition = robot.vSlide!!.currentPosition - 100

        //https://www.reddit.com/r/FTC/comments/78l5o0/how_to_program_encoders/
        telemetry.addData("Status: ", "Autonomous Terminated")
        telemetry.update()
    }

    fun pause()
    {
        robot.brake()
        sleep(400)
    }
}