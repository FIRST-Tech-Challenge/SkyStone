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
        robot.vSlide?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        robot.vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        waitForStart()
        //robot.vSlide!!.targetPosition = robot.vSlide!!.currentPosition //sets initial positon to target later
        robot.vPow(0.5)
        sleep(3000)//raise vertical slide
        robot.hSlide?.position = 0.5 //extend h slide
        sleep(500)
        //robot.vSlide?.power = 0.0
        //robot.hSlide?.position = 0.5 //Sets hSlide to no power
        robot.drive(0.5) //drive up to foundation
        sleep(3500)
        robot.brake()
        robot.vSlide?.power = -0.75 //lower vertical slide
        sleep(3000)
        robot.vSlide?.power = 0.0
        robot.rightDrive?.power = 0.5
        //robot.vSlide?.mode = DcMotor.RunMode.RUN_TO_POSITION //runs back to initial position
        //sleep(500)
        //robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //runs back to initial position
        //robot.vSlide?.power = 0.0
        robot.drive(-0.25) //drive backward
        sleep(3500)
        robot.brake()
        robot.leftDrive?.power = -0.25 //NEEDS TO BE 90 DEGREES OR SLIGHTLY MORE
        sleep(1000)
        robot.brake()
        robot.drive(0.2) //goes towards the wall
        sleep(5000)
        robot.brake()

        /* Steps
        1. Raises Linear Slide and horizontal Slide
        2. Drives forawrd to the foundation
        3. Puts the Linear Slide back to position
        4. Turn right motor to 90 degress
        5. Drive forard to the Building Zone against the wall
         */

        //https://www.reddit.com/r/FTC/comments/78l5o0/how_to_program_encoders/
        telemetry.addData("Status: ", "Autonomous Terminated")
        telemetry.update()
    }
}
