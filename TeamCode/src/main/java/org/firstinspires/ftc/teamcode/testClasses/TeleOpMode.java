package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.RobotMap.*;

import java.util.HashMap;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap, new HashMap<ChassisMotor, String>() {{
            put(ChassisMotor.FRONT_LEFT, "front_left_drive");
            put(ChassisMotor.FRONT_RIGHT, "front_right_drive");
            put(ChassisMotor.BACK_LEFT, "back_left_drive");
            put(ChassisMotor.BACK_RIGHT, "back_right_drive");
        }});
        chassis.reverseMotors(new ChassisMotor[]{
                ChassisMotor.FRONT_LEFT,
                ChassisMotor.BACK_LEFT
        });
        Hook hook = new Hook(hardwareMap, new HashMap<HookServo, String>() {{
            put(HookServo.MAIN, "hook");
        }});
        Arm arm = new Arm(hardwareMap, new HashMap<ArmMotor, String>() {{
            put(ArmMotor.MAIN, "arm");
        }}, new HashMap<ArmServo, String>() {{
            put(ArmServo.INTAKE, "intake");
            put(ArmServo.WRIST, "wrist");
        }});
        telemetry.addData("Init", "v:1.0");
        waitForStart();

        while (opModeIsActive()) {
            Controller controller = new Controller(gamepad1);
            /*
            //Intake test

            */
            runChassis(chassis, controller);
            runHook(hook, controller);
            runArm(arm, controller);
            telemetry.update();
        }
    }

    public void runArm(Arm arm, Controller controller) {
        if(controller.getX()){
            if(controller.getX()){
                positions.put(ArmServo.INTAKE, .01);
            }else if(controller.getY()){
                positions.put(ArmServo.INTAKE, -.01);
            }
            arm.runArm(new HashMap<ArmServo, Double>(), new HashMap<ArmMotor, Double>());
        }
    }

    public void runHook(Hook hook, Controller controller) {
        if (controller.getA()) {
            hook.runServo(.01);
        } else if (controller.getB()) {
            hook.runServo(-.01);
        }
    }

    public void runChassis(Chassis chassis, Controller controller) {
        final double power = Math.hypot(controller.getLeftStickX(), controller.getLeftStickY());//Flip Y stick
        final double angle = Math.atan2(controller.getLeftStickY(), controller.getLeftStickX()) - Math.PI / 4;
        final double turn = controller.getRightStickX();
        chassis.runChassis(angle, turn, power);
    }
}  
