
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Halo Drive")
public class DriveHalo extends OpMode {

    private Robot robot = new Robot();

    // init variables
    private double speedControl = 0.5; // to make the robot go slower since we use Orbital 20s
    private double compensation = 1; // compensation so the robot can move forward AND turn while both joysticks are used
    private float deadZone = 0.1f; // joystick deadzone
    private boolean buttonPressed = false;
    private boolean armClosed = false;
    private double wristPosition = 0;
    boolean slowMode = false; // activate slowMode if both joysticks are pushed down
    boolean strafeMode = false;
    Boolean[] buttons = new Boolean[7];


    @Override
    public void init() {
        robot.init(hardwareMap);
        gamepad1.setJoystickDeadzone(deadZone);
        for (int i = 0; i < buttons.length; i++) {
            buttons[i] = false;
        }
        telemetry.addData("Initialized", "Ready to start");
    }

    @Override
    public void loop() {
        this.driveController();
        this.armController();
        this.wristController();
        this.gripperController();
        this.liftController();
        this.waffleController();
        telemetry.addData("Robot Info: ", robot.getInfo());
        telemetry.update();
    }

    void liftController() {
        if (gamepad1.dpad_down) { // lift logic
            robot.liftDown();
        } else if (gamepad1.dpad_up) {
            robot.liftUp();
        } else {
            robot.stopLift();
        }
    }

    void waffleController() {
        if (gamepad1.y && !buttons[0]) {
            try {
                robot.moveWaffleMover('h');
            } catch (InterruptedException e) {
                telemetry.addData("Error", "Thread.sleep in moveWaffleMover failed");
                telemetry.update();
            }
        }
        buttons[0] = gamepad1.y;
    }

    void armController() {
        if (gamepad2.dpad_down && !buttons[1]) {
            robot.bringArmDown(this);
        } else if (gamepad2.dpad_up && !buttons[2]) {
            try {
                robot.foldArmBack(this);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        buttons[1] = gamepad2.dpad_down;
        buttons[2] = gamepad2.dpad_up;
    }

    void wristController() {
        if (gamepad2.b && !buttons[6]) {
            robot.toggleArmRotate();
        }
        buttons[6] = gamepad2.b;
    }

    void gripperController() {
        if (gamepad2.a && !buttons[5]) {
            if (armClosed) {
                robot.releaseBlock(this); // release the block
            } else {
                robot.gripBlock(); // grab the block
            }
            armClosed = !armClosed;
        }
        buttons[5] = gamepad2.a;
    }

    void driveController() {
        this.slowMode = gamepad1.left_stick_button && gamepad1.right_stick_button;
        this.strafeMode = !gamepad1.left_stick_button && gamepad1.right_stick_button;

        if (this.slowMode) {
            speedControl = 0.25;
        } else {
            speedControl = 0.5;
        }

        if (this.strafeMode) {
            robot.setStrafe(speedControl * gamepad1.right_stick_x);
        } else {
            double drive = speedControl * -gamepad1.left_stick_y; // forward
            double turn = 0.5 * speedControl * gamepad1.right_stick_x; // turn
            double leftPower = Range.clip(drive + turn + compensation * turn, -1, 1);
            double rightPower = Range.clip(drive - turn - compensation * turn, -1, 1);
            robot.rearLeft.setPower(leftPower);
            robot.frontLeft.setPower(leftPower);
            robot.rearRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
        }
    }
}
