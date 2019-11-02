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
        //robot.vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        //robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.vSlide?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        waitForStart()
        //robot.vSlide?.power = 0.60
        /*sleep(200)
        robot.drive(0.5)
        sleep(750)
        robot.claw?.position = 1.0
        robot.hSlide?.position = 0.01
        robot.hSlide?.position = 0.01
        //robot.vSlide?.power = -0.60 */
        //sleep(200)
        //robot.vSlide!!.targetPosition = robot.vSlide!!.currentPosition //sets initial positon to target later
        robot.vSlide?.power = 0.60 //raise vertical slide
        robot.hSlide?.position = 0.01 //extend h slide
        sleep(1500)
        robot.vSlide?.power = 0.0
        robot.hSlide?.position = 0.5 //Sets hSlide to no power
        robot.claw?.position = 1.0
        robot.leftDrive?.power = 0.52
        robot.rightDrive?.power = 0.5
        sleep(1500)
        robot.leftDrive?.power = 0.75
        sleep(500)
        robot.rightDrive?.power = 0.75
        sleep(450)
        robot.leftDrive?.power = 0.52
        robot.rightDrive?.power = 0.5
        sleep(150)
        robot.leftDrive?.power = -0.52
        robot.rightDrive?.power = 0.5
        sleep(100)
        robot.vSlide?.power = 0.60
        sleep(250)
        //robot.drive(0.5) //drive up to foundation
        //sleep(3000)
        robot.brake()
        //robot.hSlide?.position = 0.01
        //robot.hSlide?.position = 0.01
        robot.vSlide?.power = -0.60 //lower vertical slide
        sleep(2000)
        //robot.vSlide?.power = 0.0
        /*robot.vSlide?.mode = DcMotor.RunMode.RUN_TO_POSITION //runs back to initial position
        sleep(500)
        robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //runs back to initial position
        robot.vSlide?.power = 0.0*/
        robot.drive(-0.75) //drive backward
        sleep(3500)
        robot.brake()
        //robot.rightDrive?.power = -0.25 //NEEDS TO BE 90 DEGREES OR SLIGHTLY MORE
        //sleep(4000)
        //robot.brake()
        //robot.drive(0.2) //goes towards the wall
        //sleep(5000)
        //robot.brake()

        /* Steps
        1. Raises Linear Slide and horizontal Slide
        2. Drives forawrd to the foundation
         */

        //https://www.reddit.com/r/FTC/comments/78l5o0/how_to_program_encoders/
        telemetry.addData("Status: ", "Autonomous Terminated")
        telemetry.update()
    }
}
