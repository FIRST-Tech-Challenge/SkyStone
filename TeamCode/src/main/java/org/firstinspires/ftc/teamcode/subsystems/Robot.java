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
        //arm = new Arm(hardwareMap);
        //hook = new Hook(hardwareMap);
        chassis = new Chassis(hardwareMap);
        //intake = new Intake(hardwareMap);
    }

    public void run(Controller controller) {
        final double leftStickX = controller.limitStick(controller.getLeftStickX());
        final double leftStickY = controller.limitStick(controller.getLeftStickY());
        final double rightStickX = controller.limitStick(controller.getRightStickX());
        final double power = Math.hypot(leftStickX, leftStickY);
        final double targetAngle = Math.atan2(leftStickY, leftStickX);
        final double turn = rightStickX;
        chassis.run(targetAngle, turn, power);
        //Arm check
        if (controller.getRightBumper() != controller.rightBumperLast) {
            if (controller.getRightBumper()) {
                level++;
                controller.rightBumperLast = true;
            } else {
                controller.rightBumperLast = false;
            }
        }
        if (controller.getLeftBumper() != controller.leftBumperLast) {
            if (controller.getLeftBumper()) {
                level--;
                controller.leftBumperLast = true;
            } else {
                controller.leftBumperLast = false;
            }
        }
        arm.run(level);
        //
        while(setLevel(level));
        //Put in all run methods
        //hook.run();
        //
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
        chassis.run(targetAngle, turn, power);
        //intake.run();
    }
    public void checkCommands(){

    }

    public boolean setLevel(int level){
        //init command list here
        return true;
    }
}
