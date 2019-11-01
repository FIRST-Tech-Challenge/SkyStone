package org.firstinspires.ftc.teamcode



import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor


/**
 * Created by KasaiYuki on 9/21/2018.
 */
@Autonomous(name = "SSAutoOp", group = "Autonomous")
//@Disabled
class SSAutoOp : LinearOpMode()
{
    val robot = SSRobot()

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        telemetry.addData("Status: ", "Autonomous Initialized")
        telemetry.update()

        robot.init(hardwareMap)
        robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        waitForStart()
        robot.vSlide!!.targetPosition = robot.vSlide!!.currentPosition //sets initial positon to target later
        robot.vSlide?.power = 0.25 //raise vertical slide
        robot.hSlide?.position = 0.05 //extend h slide
        sleep(500)
        robot.vSlide?.power = 0.0
        robot.hSlide?.position = 0.5
        robot.drive(0.5) //drive up to foundation
        sleep(2000)
        robot.brake()
        /*robot.vSlide?.power = -0.25 //lower vertical slide
        sleep(500)
        robot.vSlide?.power = 0.0*/
        robot.vSlide?.mode = DcMotor.RunMode.RUN_TO_POSITION //runs back to initial position
        sleep(500)
        robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //runs back to initial position
        robot.vSlide?.power = 0.0
        robot.drive(-0.25) //drive backward
        sleep(3500)
        robot.brake()
        robot.rightDrive?.power = -0.25 //NEEDS TO BE 90 DEGREES OR SLIGHTLY MORE
        sleep(550)
        robot.brake()
        robot.drive(0.2) //goes towards the wall
        sleep(5000)
        robot.brake()

        //https://www.reddit.com/r/FTC/comments/78l5o0/how_to_program_encoders/
        telemetry.addData("Status: ", "Autonomous Terminated")
        telemetry.update()
    }
}
