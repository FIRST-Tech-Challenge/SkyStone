package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor


/**
 * Created by KasaiYuki on 9/21/2018.
 */
@Autonomous(name = "SSBuildRedAuto", group = "Autonomous")
//@Disabled
class SSBuildRedAuto : LinearOpMode()
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
/*      robot.strafe(1.0) //align with foundation
        sleep(800)*/
        //Positive Value = Left Strafe || Negative Value = Right Strafe
        robot.strafe(1.0) //align with foundation
        sleep(500)
        robot.brake()
        sleep(100)
        robot.strafe(0.5)
        sleep(800)
        robot.drive(0.50) //Drives Forward to the Foundation
        sleep(1575)
        robot.brake()
        sleep(1000)
        robot.leftHook?.position = 0.72 // Grabs Onto the Foundation
        robot.rightHook?.position = 0.72
        sleep(500)
        robot.drive(-0.05)
        sleep(1000)
        robot.brake()
        robot.drive(-0.50) //Drives back with the foundation
        sleep(4000)
        robot.brake()
        robot.leftHook?.position = 0.0 // Grabs Onto the Foundation
        robot.rightHook?.position = 0.0
        robot.strafe(0.5)
        sleep(750)
        robot.brake()
        robot.drive(0.5)
        sleep(500)
        robot.brake()
        robot.strafe(-0.5)
        sleep(750)
        robot.brake()
        robot.drive(-0.5)
        sleep(2000)
        robot.brake()


        //https://www.reddit.com/r/FTC/comments/78l5o0/how_to_program_encoders/
        telemetry.addData("Status: ", "Autonomous Terminated")
        telemetry.update()
    }
}
