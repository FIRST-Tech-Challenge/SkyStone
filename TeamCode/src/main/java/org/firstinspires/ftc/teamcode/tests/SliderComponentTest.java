package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.ArmSystem;

import java.util.EnumMap;

@TeleOp(name="SliderComponentTest", group="TeleOp")
public class SliderComponentTest extends OpMode {

    private ArmSystem armSystem;
    // Use the following variables to detect if their respective bumpers have been pressed the
    // previous loop. Otherwise, hitting a bumper will increase the queued height by a like 30.
    boolean m_right = false;
    boolean m_left = false;
    boolean m_gripper = false; // Gripper button`

    public void init() {
        EnumMap<ArmSystem.ServoNames, Servo> servoEnumMap = new EnumMap<ArmSystem.ServoNames, Servo>(ArmSystem.ServoNames.class);
        servoEnumMap.put(ArmSystem.ServoNames.GRIPPER, hardwareMap.get(Servo.class, "gripper"));
        servoEnumMap.put(ArmSystem.ServoNames.ELBOW, hardwareMap.get(Servo.class, "elbow"));
        servoEnumMap.put(ArmSystem.ServoNames.WRIST, hardwareMap.get(Servo.class, "wrist"));
        servoEnumMap.put(ArmSystem.ServoNames.PIVOT, hardwareMap.get(Servo.class, "pivot"));
        armSystem = new ArmSystem(
                servoEnumMap,
                hardwareMap.get(DcMotor.class, "slider_motor"),
                hardwareMap.get(DigitalChannel.class, "slider_switch"));
    }

    public void init_loop() {
        if (!armSystem.isCalibrated()) {
            armSystem.calibrate();
        } else {
            telemetry.addData("", "Calibrated!");
        }
        telemetry.addData("", armSystem.getSwitchState());
        telemetry.update();
        try {
            Thread.sleep(2);
        } catch (Exception e) {

        }
    }

    public void start() {
        armSystem.setSliderHeight(1);
    }

    public void loop() {
        // Put every joystick value to the 3rd power for greater control over the robot
        // 1^3 = 1, so we don't even need to trim the values or anything
        float rx = (float) Math.pow(gamepad1.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 3);

        if (gamepad1.dpad_left) {
            armSystem.movePresetPosition(ArmSystem.Position.POSITION_WEST);
        } else if (gamepad1.dpad_up) {
            armSystem.movePresetPosition(ArmSystem.Position.POSITION_HOME);
        } else if (gamepad1.dpad_down) {
            armSystem.movePresetPosition(ArmSystem.Position.POSITION_SOUTH);
        } else if (gamepad1.dpad_right) {
            armSystem.movePresetPosition(ArmSystem.Position.POSITION_EAST);
        }

        if (gamepad1.right_bumper && !m_right) {
            armSystem.queuedHeight++;
            armSystem.go();
            m_right = true;
        } else if (!gamepad1.right_bumper) {
            m_right = false;
        }

        if (gamepad1.left_bumper && !m_left) {
            armSystem.queuedHeight--;
            armSystem.go();
            m_left = true;
        } else if (!gamepad1.left_bumper) {
            m_left = false;
        }

        if (gamepad1.a && !m_gripper) { // This doesn't fit the spec on the google doc and should be changed later
            armSystem.toggleGripper();
            m_gripper = true;
        } else if (!gamepad1.a) {
            m_gripper = false;
        }
        /*
        // Display queued positions on telemetry data so that the captain can call them out to the
        // driver
        if (!(armSystem.queuedPosition == null)) {
            telemetry.addData("",
                    "Queued position: " + armSystem.queuedPosition.toString());
        }

        telemetry.addData("", "Queued height: " + armSystem.queuedHeight);
        telemetry.addData("", armSystem.targetHeight);

        if (gamepad1.y) { // "yeet" it as Brian would say
            armSystem.go();
        }
         */
        telemetry.update();

        armSystem.updateHeight();
    }
}
