
package org.firstinspires.ftc.teamcode;
//test
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Halo Drive")
public class DriveHalo extends OpMode {

    Robot robot = new Robot();

    enum TeleOpMode {STRAFE, DRIVE, WAFFLE_MOVER}

    TeleOpMode driveMode;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Initialized", "Ready to start");
    }

    @Override
    public void loop() {
        double speedControl = 0.5; // to make the robot go slower since we use Orbital 20s
        boolean slowMode = gamepad1.left_stick_button && gamepad1.right_stick_button; // activate slowMode if both joysticks are pushed down
        boolean strafeMode = !gamepad1.left_stick_button && gamepad1.right_stick_button;
        double compensation = 0.2; // compensation so the robot can move forward AND turn while both joysticks are used
        double wafflePower = 0; // power on the waffle motor
        float deadZone = 0.1f; // joystick deadzone

        if (slowMode) {
            speedControl = 0.25;
        }


        gamepad1.setJoystickDeadzone(deadZone);
        if (strafeMode) {
            driveMode = TeleOpMode.STRAFE;
        } else if (gamepad1.a) {
            driveMode = TeleOpMode.WAFFLE_MOVER;
        } else {
            driveMode = TeleOpMode.DRIVE;
        }

        double drive = speedControl * -gamepad1.left_stick_y;
        double turn = speedControl * gamepad1.right_stick_x;

        switch (driveMode) {
            case DRIVE:
                double leftPower = Range.clip(drive + turn + compensation * turn, -1, 1);
                double rightPower = Range.clip(drive - turn - compensation * turn, -1, 1);
                robot.rearLeft.setPower(leftPower);
                robot.frontLeft.setPower(leftPower);
                robot.rearRight.setPower(rightPower);
                robot.frontRight.setPower(rightPower);
                break;

            case STRAFE:
                robot.setStrafe(speedControl * gamepad1.right_stick_x);
                break;

            case WAFFLE_MOVER:
                try {
                    robot.moveWaffleMover('f');
                } catch (InterruptedException e) {
                    telemetry.addData("Error", "Thread.sleep in moveWaffleMover failed");
                    telemetry.update();
                }
        }
    }
}