
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Halo Drive")
public class DriveHalo extends OpMode {

    private Robot robot = new Robot();

    // init variables
    private double speedControl = 0.5; // to make the robot go slower since we use Orbital 20s
    private double compensation = 0.2; // compensation so the robot can move forward AND turn while both joysticks are used
    private float deadZone = 0.1f; // joystick deadzone
    private boolean buttonPressed = false;
    private boolean armClosed = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Initialized", "Ready to start");
    }

    @Override
    public void loop() {
        buttonPressed = gamepad1.y || gamepad1.dpad_down || gamepad1.dpad_up || gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.a;
        boolean slowMode = gamepad1.left_stick_button && gamepad1.right_stick_button; // activate slowMode if both joysticks are pushed down
        boolean strafeMode = !gamepad1.left_stick_button && gamepad1.right_stick_button;

        if (slowMode) {
            speedControl = 0.25;
        }

        gamepad1.setJoystickDeadzone(deadZone);

        if (strafeMode) {
            robot.setStrafe(speedControl * gamepad1.right_stick_x);
        } else {
            double drive = speedControl * -gamepad1.left_stick_y; // forward
            double turn = speedControl * gamepad1.right_stick_x; // turn
            double leftPower = Range.clip(drive + turn + compensation * turn, -1, 1);
            double rightPower = Range.clip(drive - turn - compensation * turn, -1, 1);
            robot.rearLeft.setPower(leftPower);
            robot.frontLeft.setPower(leftPower);
            robot.rearRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
        }
        if (!buttonPressed) { // button mapping
            if (gamepad1.y) {
                try {
                    robot.moveWaffleMover('h');
                } catch (InterruptedException e) {
                    telemetry.addData("Error", "Thread.sleep in moveWaffleMover failed");
                    telemetry.update();
                }
            }

            if (gamepad1.dpad_down) {
                try {
                    robot.bringArmDown(this);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else if (gamepad1.dpad_up) {
                try {
                    robot.foldArmBack(this);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            if (gamepad2.dpad_down) { // lift logic
                robot.gripperDown();
            } else if (gamepad2.dpad_up) {
                robot.gripperUp();
            } else {
                robot.stopLift();
            }

            if (gamepad2.a) {
                if (armClosed) {
                    try { // release the block
                        robot.releaseBlock(this, false);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                } else {
                    robot.gripBlock(); // grab the block
                }
                armClosed = !armClosed;
            }
            buttonPressed = !buttonPressed;
        }
    }
}