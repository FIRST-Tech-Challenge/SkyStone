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
}
