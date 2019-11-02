package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GripperTests")
public class GripperTests extends OpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            try {
                robot.bringArmDown(this);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
