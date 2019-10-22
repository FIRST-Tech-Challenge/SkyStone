package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Hook;


import java.util.HashMap;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        telemetry.addData("Init", "v:1.0");
        waitForStart();

        while (opModeIsActive()) {
            Controller controller = new Controller(gamepad1);
            /*
            //Intake test

            */
            runChassis(chassis, controller);

            telemetry.update();
        }
    }

    public void runArm(Arm arm, Controller controller) {

    }

    public void runHook(Hook hook, Controller controller) {

    }

    public void runChassis(Chassis chassis, Controller controller) {
        final double power = Math.hypot(controller.getLeftStickX(), controller.getLeftStickY());//Flip Y stick
        final double angle = Math.atan2(controller.getLeftStickY(), controller.getLeftStickX()) - Math.PI / 4;
        final double turn = controller.getRightStickX();
        chassis.runChassis(angle, turn, power);
    }
}  
