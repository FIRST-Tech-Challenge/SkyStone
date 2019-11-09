package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Arm Control")
public class ArmControl extends OpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        try {
            robot.init(this);
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Slenderman messed up our robot\n" + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        robot.setArmRotatePower(-gamepad2.left_stick_y);
    }
}
