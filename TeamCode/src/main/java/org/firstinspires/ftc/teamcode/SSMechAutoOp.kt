package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor


/**
 * Created by KasaiYuki on 9/21/2018.
 */
@Autonomous(name = "SSMechAutoOp", group = "Autonomous")
//@Disabled
class SSMechAutoOp : LinearOpMode()
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
        robot.drive(0.50) //Drives Forward to the Foundation
        sleep(2100)
        robot.brake()
        sleep(1000)
        robot.leftHook?.position = 0.72 // Grabs Onto the Foundation
        robot.rightHook?.position = 0.72
        sleep(500)
        robot.drive(-0.50) //Drives back with the foundation
        sleep(1500)
        robot.brake()
        robot.leftPow(0.25)
        robot.rightPow(-0.25) //Turn 90 degrees and drives it up to the wall in the building zone
        sleep(7000)
        robot.drive(0.5)
        sleep(1500)
        robot.brake()
        robot.leftHook?.position = 0.0 // Puts the Hooks back
        robot.rightHook?.position = 0.0
        robot.drive(-0.50) //Parks Under Bridge
        sleep(1500)
        /*robot.vSlide?.power = -0.60
        //sleep(200)
        //robot.vSlide!!.targetPosition = robot.vSlide!!.currentPosition //sets initial positon to target later
        //robot.vSlide?.power = 0.60 //raise vertical slide
        //robot.hSlide?.position = 0.01 //extend h slide
        //sleep(1500)
        //robot.claw?.position = 1.0
        robot.drive(0.5)
        sleep(3000)
        robot.vSlide?.power = -0.60
        sleep(2000)
        robot.leftPow?.power = 0.75
        sleep(500)
        robot.rightPow?.power = 0.75
        sleep(450)
        robot.leftDrive?.power = 0.52
        robot.rightDrive?.power = 0.5
        sleep(150)
        robot.leftDrive?.power = -0.52
        robot.rightDrive?.power = 0.5
        sleep(100)
        robot.vSlide?.power = -0.60
        sleep(2000)
        /*robot.vSlide?.mode = DcMotor.RunMode.RUN_TO_POSITION //runs back to initial position
        sleep(500)
        robot.vSlide?.mode = DcMotor.RunMode.RUN_USING_ENCODER //runs back to initial position
        robot.vSlide?.power = 0.0*/
        robot.drive(-0.75) //drive backward
        sleep(3500)
        robot.brake() */


        //https://www.reddit.com/r/FTC/comments/78l5o0/how_to_program_encoders/
        telemetry.addData("Status: ", "Autonomous Terminated")
        telemetry.update()
    }
}
