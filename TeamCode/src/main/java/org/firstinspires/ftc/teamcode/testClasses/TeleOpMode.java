package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


import java.util.HashMap;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Controller controller = new Controller(gamepad1);
        Robot robot = new Robot(hardwareMap);
        telemetry.addData("Init", "v:1.0");
        waitForStart();
        while (opModeIsActive()) {
            robot.run(controller);
            telemetry.addData("test1: ", gamepad1.b);
            telemetry.addData("test: :", controller.b);
            telemetry.addData("test3: :", controller.getB());
            telemetry.addData("arm level: ", robot.level);
            telemetry.addData("arm pos: ", robot.arm.main.getTargetPosition());
            telemetry.update();
        }
    }

}  
