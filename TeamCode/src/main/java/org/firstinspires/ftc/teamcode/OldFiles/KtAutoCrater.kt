package org.firstinspires.ftc.teamcode.OldFiles


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode


/**
 * Created by KasaiYuki on 9/21/2018.
 */

@Autonomous(name="KtAutoCrater", group = "Autonomous")
@Disabled
class KtAutoCrater : LinearOpMode()
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
        //runtime.reset()
        robot.drop()
        robot.drive(0.5)
        sleep(1000)

    }

}