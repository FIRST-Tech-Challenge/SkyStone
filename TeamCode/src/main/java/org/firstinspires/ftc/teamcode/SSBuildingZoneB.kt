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
        waitForStart()
        robot.leftHook?.position = 0.0
        robot.rightHook?.position = 0.0
        robot.vSlide?.targetPosition = 100 + robot.vSlide!!.currentPosition
        robot.vSlide?.power = 0.1
        pause()
        robot.drive(0.50) //Drives Forward to the Stones
        sleep(1550)
        pause()
        robot.claw?.position = 0.0
        pause()
        sleep(500)
        robot.drive(-0.50)
        sleep(350)
        pause()
        robot.strafe(1.0)//Heads to Foundation
        sleep(3550)
        robot.drive(0.5)
        sleep(250)
        pause()
        robot.claw?.position = robot.clawPinchPos
        pause()
        robot.strafe(0.5)
        sleep(1000)
        pause()
        robot.vSlide?.targetPosition = robot.vSlide!!.currentPosition - 100

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
