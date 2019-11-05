package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


import java.util.HashMap;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        telemetry.addData("Init", "v:1.0");
        waitForStart();
        while (opModeIsActive()) {
            Controller controller = new Controller(gamepad1);
            robot.run(controller);
            telemetry.addData("arm motor: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}  
