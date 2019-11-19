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
        hook = new Hook(hardwareMap);
        chassis = new Chassis(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void run(Controller controller) {
        //Chassis get

        final double leftStickX = controller.sensitiveStick(controller.getLeftStickX());
        final double leftStickY = controller.sensitiveStick(controller.getLeftStickY());
        final double rightStickX = controller.sensitiveStick(controller.getRightStickX());
        final double power = Math.hypot(leftStickX, leftStickY);
        final double targetAngle = Math.atan2(leftStickY, leftStickX);
        final double turn = rightStickX;
        //Arm get
        //Put in all run methods
        setLevel(controller);
        arm.run(level);
        chassis.run(targetAngle, turn, power);
        if (controller.getB()) {
            intake.setMainToHighPosition();
        }
        if (controller.getA()) {
            intake.setMainToLowPosition();
        }
        //hook.run();
        //intake.run();
    }

    public void test(double rot, double power){
        chassis.runRotations(rot, power);
    }


    public void setArm(Arm arm) {
        this.arm = arm;
    }

    public void setHook(Hook hook) {
        this.hook = hook;
    }

    public void setChassis(Chassis chassis) {
        this.chassis = chassis;
    }

    public void setIntake(Intake intake) {
        this.intake = intake;
    }

    public void setLevel(Controller controller) {
        if (controller.getRightBumper()) {
            level++;
            if (level == arm.levelAngles.length) {
                level = arm.levelAngles.length-1;
            }
            while (controller.getRightBumper()) ;
        }
        if (controller.getLeftBumper()) {
            level--;
            if (level < 0) {
                level = 0;
            }
            while (controller.getLeftBumper()) ;
        }
    }
}
