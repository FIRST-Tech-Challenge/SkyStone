package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Hook;


import java.util.HashMap;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        telemetry.addData("Init", "v:1.0");
        waitForStart();

        while (opModeIsActive()) {
            Controller controller = new Controller(gamepad1);
            runChassis(chassis, controller);
            runArm(arm, controller);
            /*
            HashMap<String, Integer> chassisMotorPositions = chassis.getMotorPositions();
            for(HashMap.Entry<String, Integer> chassisMotorPosition : chassisMotorPositions.entrySet()){
               telemetry.addData(chassisMotorPosition.getKey(), chassisMotorPosition.getValue());
            }
            */
            telemetry.addData("arm motor: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void runArm(Arm arm, Controller controller) {
            if (controller.getA())
                arm.reset();
        if (controller.getY())
            arm.getMain().setPower(.1);
        else
            arm.getMain().setPower(0);
        telemetry.addData("Arm: ", arm.getMain().getCurrentPosition());
    }

    public void runHook(Hook hook, Controller controller) {

    }

    public void runChassis(Chassis chassis, Controller controller) {
        final double leftStickX = controller.limitStick(controller.getLeftStickX());
        final double leftStickY = controller.limitStick(controller.getLeftStickY());
        final double rightStickX = controller.limitStick(controller.getRightStickX());
        final double power = Math.hypot(leftStickX, leftStickY);
        final double angle = Math.atan2(leftStickY, leftStickX);
        final double turn = rightStickX;
        chassis.runChassis(angle, turn, power);
    }
}  
