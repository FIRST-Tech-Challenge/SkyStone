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
    val robot = SSMechRobot()

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        telemetry.addData("Status: ", "Autonomous Initialized")
        telemetry.update()

        robot.init(hardwareMap)
        //robot.vSlide?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        //robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.vSlide?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        waitForStart()
        robot.leftHook?.position = 0.0
        robot.rightHook?.position = 0.0
/*        robot.strafe(1.0) //align with foundation
        sleep(800)*/
        robot.brake()
/*        robot.bLDrive?.power = (-0.5) //left
        robot.bRDrive?.power = (0.5)
        robot.fLDrive?.power = (0.5)
        robot.fRDrive?.power = (-0.5)*/
        robot.strafe(0.5)

        //br == -      RIGHT
        //bl ==
        //fr ==
        //fl == -
        sleep(4000)
        /*sleep(100)
        robot.strafe(1.0)
        sleep(750)*/
        robot.drive(0.50) //Drives Forward to the Foundation
        sleep(2500)
        robot.brake()


        //https://www.reddit.com/r/FTC/comments/78l5o0/how_to_program_encoders/
        telemetry.addData("Status: ", "Autonomous Terminated")
        telemetry.update()
    }
}
