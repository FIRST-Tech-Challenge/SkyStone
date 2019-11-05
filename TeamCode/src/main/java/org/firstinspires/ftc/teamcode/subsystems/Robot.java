package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    // calling other classes
    public Arm arm;
    public Hook hook;
    public Chassis chassis;
    public Intake intake;

    public Robot(HardwareMap hardwareMap) {
        //constructors
        arm = new Arm(hardwareMap);
        hook = new Hook(hardwareMap);
        chassis = new Chassis(hardwareMap);
        intake = new Intake(hardwareMap);
    }
    public void run(Controller controller){
        final double leftStickX = controller.limitStick(controller.getLeftStickX());
        final double leftStickY = controller.limitStick(controller.getLeftStickY());
        final double rightStickX = controller.limitStick(controller.getRightStickX());
        final double power = Math.hypot(leftStickX, leftStickY);
        final double targetAngle = Math.atan2(leftStickY, leftStickX);
        final double turn = rightStickX;
        //Put in all run methods
        //arm.run();
        hook.run();
        //
        chassis.run(targetAngle, turn, power);
        intake.run();
    }
}
