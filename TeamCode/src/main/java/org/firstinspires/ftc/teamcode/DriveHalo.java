package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
//test
public class DriveHalo extends OpMode {

    Robot robot = new Robot();
    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Initialized", "Ready to start");
    }

    @Override
    public void loop() {
        double speedControl = 0.75; // to make the robot go slower since we use Orbital 20s
        boolean slowMode = gamepad1.left_stick_button && gamepad1.right_stick_button; // activate slowMode if both joysticks are pushed down
        boolean strafeMode = !gamepad1.left_stick_button && gamepad1.right_stick_button;
        double compensation = 0.2; // compensation so the robot can move forward AND turn while both joysticks are used

        float deadZone = 0.1f;
        double drive = speedControl * -gamepad1.left_stick_y;
        double turn = speedControl * -gamepad1.right_stick_x;

        if (slowMode) {
            speedControl = 0.5;
        }

        gamepad1.setJoystickDeadzone(deadZone);

        // strafe if only right stick button is down
        if (strafeMode) {
            robot.rearLeft.setPower(-turn);
            robot.frontRight.setPower(-turn);

            robot.frontLeft.setPower(turn);
            robot.rearRight.setPower(turn);
        }
        // Halo drive
        else {
            double leftPower = Range.clip(drive + turn + compensation * turn, -1, 1);
            double rightPower = Range.clip(drive - turn - compensation * turn, -1, 1);
            robot.rearLeft.setPower(leftPower);
            robot.frontLeft.setPower(leftPower);

            robot.rearRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
        }
    }
}
