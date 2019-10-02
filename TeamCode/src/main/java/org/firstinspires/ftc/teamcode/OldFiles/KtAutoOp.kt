package org.firstinspires.ftc.teamcode.OldFiles



import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode


/**
 * Created by KasaiYuki on 9/21/2018.
 */

@Autonomous(name="KtAutoOp", group = "Autonomous")
@Disabled
class KtAutoOp : LinearOpMode()
{
    //adds robot objects for variables and methods
    val robot = KtRobot()

    @Throws(InterruptedException::class)
    override fun runOpMode()
    {
        telemetry.addData("Status", "Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap)

        waitForStart()
        //runtime.reset()//IDK WHY RUNTIME DOESN'T WORK!

        robot.drive(0.5)
        sleep(6000)
        robot.drive(-1.0)
        sleep(500)
        robot.drive(0.25)
        sleep(500)
        robot.drive(-1.0)
        sleep(2000)
       /* robot.liftRobot()
        sleep(2000)
        robot.drive(0.0)
        sleep(1000)
        robot.drive(35.0)
        sleep(2500)
        robot.dropToken()
        sleep(1000)
        robot.liftToken()*/


    }

}