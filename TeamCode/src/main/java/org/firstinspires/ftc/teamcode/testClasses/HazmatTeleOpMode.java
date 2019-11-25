package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.*;

/**
 * TeleOpMode for team Hazmat
 */
@TeleOp(name = "HazmatTeleOpMode", group = "Teleop")
public class HazmatTeleOpMode extends LinearOpMode{
    public void runOpMode() {
        //Instantiate Controller and Robot
        Controller controller = new Controller(gamepad1);
        Robot robot = new Robot(hardwareMap);

        telemetry.addData("Init", "v:1.0");
        //Wait for pressing plan on controller
        waitForStart();

        //Run Robot based on Tonctoller inputs
        while (opModeIsActive()) {
            robot.run(controller);
            telemetry.update();
            }
    }
}
