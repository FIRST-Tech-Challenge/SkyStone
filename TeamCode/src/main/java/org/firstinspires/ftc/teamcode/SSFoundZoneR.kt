package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor


/**
 * Created by KasaiYuki on 9/21/2018.
 */
@Autonomous(name = "SSFoundZoneR", group = "Autonomous")
//@Disabled
class SSFoundZoneR : LinearOpMode()
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
        waitForStart()
        robot.leftHook?.position = 0.0
        robot.rightHook?.position = 0.0
        robot.vSlide?.targetPosition = 90 + robot.vSlide!!.currentPosition
        robot.vSlide?.power = 0.5
        robot.strafe(-0.75) //align with foundation
        sleep(500)
        pause()
        robot.drive(0.50) //Drives Forward to the Foundation
        sleep(1100)
        pause()
        robot.strafe(1.0)
        sleep(3550)
        pause()
        robot.claw?.position = robot.clawPinchPos
        sleep(1000)
        robot.drive(0.5)
        sleep(800)
        pause()
        robot.claw?.position = 0.0
        pause()
        robot.drive(-0.4)
        sleep(750)
        pause()
        robot.strafe(-1.0)
        sleep(2700)
        pause()
/*        robot.vSlide?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        robot.vSlide?.power = 0.5
        sleep(1000)
        robot.claw?.position = 0.0
        robot.vSlide?.power = -0.5
        sleep(1000)*/
        robot.vSlide?.targetPosition = robot.vSlide!!.currentPosition + 3000
        sleep(1500)
        robot.strafe(-1.0)
        sleep(800)
        pause()
        //robot.vSlide?.targetPosition = robot.vSlide!!.currentPosition - 100
        robot.claw?.position = robot.clawPinchPos
        sleep(1000)
        robot.vSlide?.targetPosition = robot.vSlide!!.currentPosition - 3000
        robot.strafe(1.0)
        sleep(2000)
/*        robot.leftHook?.position = 0.7
        robot.rightHook?.position = 0.72
        robot.drive(0.5)
        sleep(110)
        pause()
        robot.drive(-1.0) //Drives Back the Foundation
        sleep(1100)
        pause()
        robot.strafe(0.5)
        sleep(1000)
        pause() */
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
