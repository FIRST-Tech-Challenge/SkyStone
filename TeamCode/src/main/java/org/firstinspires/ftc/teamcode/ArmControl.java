package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm Control")
public class ArmControl extends DriveHalo {

    @Override
    public void loop() {
        //this.driveController();
        this.armController();
        this.gripperController();
        //this.liftController();
        //this.waffleController();
        telemetry.addData("Robot", robot.getInfo() + "\nEncoder counts arm should be at: 3100");
        telemetry.update();
    }

    @Override
    void armController() {
        robot.setArmRotatePower(0.4 * gamepad2.left_stick_y);
        if (gamepad2.a) {
            robot.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            robot.armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
