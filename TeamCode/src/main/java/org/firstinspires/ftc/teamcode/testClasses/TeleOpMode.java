package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.*;


import java.util.HashMap;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Controller controller = new Controller(gamepad1);
        Robot robot = new Robot(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);
        chassis.reverseMotors(new DcMotor[]{chassis.frontRight, chassis.backRight});
        robot.setChassis(chassis);
        robot.intake.setwrist(1);
        telemetry.addData("Init", "v:1.0");
        waitForStart();
        while (opModeIsActive()) {
            if (controller.getA()) {
                telemetry.addData("thisn","hi");
                telemetry.update();
                telemetry.addData("thing:", robot.chassis.getAverageMotorError());
                telemetry.update();
            }
            robot.run(controller);
            telemetry.update();
        }
    }

}  
