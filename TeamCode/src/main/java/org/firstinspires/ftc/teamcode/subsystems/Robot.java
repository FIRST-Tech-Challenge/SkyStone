package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autoRes.commands.CommandList;

public class Robot {
    public int level = 0;
    // calling other classes
    public Arm arm;
    public Hook hook;
    public Chassis chassis;
    public Intake intake;
    public CommandList commands;

    public Robot(HardwareMap hardwareMap) {
        //constructors
        arm = new Arm(hardwareMap);
        //hook = new Hook(hardwareMap);
        chassis = new Chassis(hardwareMap);
        //intake = new Intake(hardwareMap);
    }

    public void run(Controller controller) {
        //Chassis get
        final double leftStickX = controller.limitStick(controller.getLeftStickX());
        final double leftStickY = controller.limitStick(controller.getLeftStickY());
        final double rightStickX = controller.limitStick(controller.getRightStickX());
        final double power = Math.hypot(leftStickX, leftStickY);
        final double targetAngle = Math.atan2(leftStickY, leftStickX);
        final double turn = rightStickX;
        if (controller.getA()) {
            level = 0;
        }
        if (controller.getB()) {
            level = 1;
        }
        if (controller.getX()) {
            level = 2;
        }
        if (controller.getY()) {
            level = 3;
        }
        if (controller.getLeftBumper()) {
            level = 4;
        }
        if (controller.getRightBumper()) {
            level = 5;
        }
        //Arm get
        //Put in all run methods
        arm.run(level);
        chassis.run(targetAngle, turn, power);
        //hook.run();
        //intake.run();
    }

    public void test(Controller controller, double powerFactor) {
        final double leftStickX = controller.limitStick(controller.getLeftStickX());
        final double leftStickY = controller.limitStick(controller.getLeftStickY());
        final double rightStickX = controller.limitStick(controller.getRightStickX());
        final double power = Math.hypot(leftStickX, leftStickY) * powerFactor;
        final double targetAngle = Math.atan2(leftStickY, leftStickX);
        final double turn = rightStickX;
        //Put in all run methods
        //arm.run();
        //hook.run();
        //
        //intake.run();
    }

    public void checkCommands() {

    }

    public void setLevel(Controller controller) {
        if (controller.right_bumper) {
            level++;
            while (controller.right_bumper) ;
        }
        if (controller.left_bumper) {
            level--;
            while (controller.left_bumper) ;
        }
    }
}
