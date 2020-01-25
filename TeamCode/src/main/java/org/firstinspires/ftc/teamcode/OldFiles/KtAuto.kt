package org.firstinspires.ftc.teamcode


/**
 * Created by KasaiYuki on 2/21/2019.
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.OldFiles.KtRobot


@Autonomous(name="KtAuto", group = "Autonomous")
@Disabled
class KtAuto : LinearOpMode()
{
    //adds robot objects for variables and methods
    val robot = KtRobot()
    var dropTime: Long = 8100

    @Throws(InterruptedException::class)
    override fun runOpMode()
    {
        telemetry.addData("Status", "Initialized")
        telemetry.update()
        //initializes all parts
        robot.init(hardwareMap)

        waitForStart()
        //runtime.reset()//IDK WHY RUNTIME DOESN'T WORK!

        robot.swing(0.15)
        //robot.liftRobot(1.0)//dropping down
        while(robot.touch!!.state)
            robot.liftRobot(1.0)//dropping down
        //sleep(dropTime)
        robot.liftRobot(0.0)

        //wiggle out
        robot.leftDrive?.power = 0.35
        robot.rightDrive?.power = -0.25
        sleep(250)
        robot.brake()

        sleep(1000)
        drive(0.25, 250)//leave hook
        robot.leftDrive?.power = -0.315 //re align
        robot.rightDrive?.power = 0.25
        sleep(250)
        drive(0.5, 5500)//drive forward
        robot.flag?.position = 0.4//lower flag
        drive(-1.0, 500)//jerk back to drop token
        drive(0.25, 500)//drive forward to push token
        robot.brake()
        robot.flag?.position = 0.15
        drive(-1.0, 750)//drive back
        robot.brake()

    }

    fun drive(pow: Double, sleepT: Long)
    {
        robot.drive(pow)
        sleep(sleepT)
    }

}