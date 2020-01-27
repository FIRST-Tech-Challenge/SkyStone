package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor


/**
 * Created by KasaiYuki on 9/21/2018.
 */
@Autonomous(name = "SSBuildRedAuto", group = "Autonomous")
@Disabled
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
        sleep(450)
        robot.brake()
        sleep(100)
        robot.drive(0.50) //Drives Forward to the Foundation
        sleep(1550)
        robot.brake()
        sleep(1000)
        robot.leftHook?.position = 0.72 // Grabs Onto the Foundation
        robot.rightHook?.position = 0.72
        sleep(500)
        robot.drive(-0.05)
        sleep(1000)
        robot.brake()
        robot.drive(-0.50) //Drives back with the foundation
        sleep(3000)
        robot.brake()
        //robot.leftPow(0.5)
        sleep(700)
        robot.brake()
        robot.leftHook?.position = 0.0 // Lets go of the Foundation
        robot.rightHook?.position = 0.0
        robot.strafe(-1.0)
        sleep(1500)
        robot.drive(0.5)
        sleep(2000)
        robot.strafe(1.0)
        sleep(1250)
        robot.drive(-0.5)
        sleep(3000)
        /*robot.leftPow(0.25)
        robot.rightPow(-0.25) //Turn 90 degrees and drives it up to the wall in the building zone
        sleep(4200)
        robot.drive(0.5) //runs into the zone
        sleep(1700)
        robot.brake()
        robot.leftHook?.position = 0.0 // Puts the Hooks back
        robot.rightHook?.position = 0.0
        robot.drive(-0.2)
        sleep(1500)*/

        //https://www.reddit.com/r/FTC/comments/78l5o0/how_to_program_encoders/
        telemetry.addData("Status: ", "Autonomous Terminated")
        telemetry.update()
    }
}
