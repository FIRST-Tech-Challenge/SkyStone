package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Troy Drive Halo")
public class DriveHaloTroy extends DriveHalo {

    @Override
    void liftController() {
        if (gamepad1.dpad_up) {
            robot.liftUp();
        } else if (gamepad1.dpad_down) {
            robot.liftDown();
        } else {
            robot.stopLift();
        }
    }
}
