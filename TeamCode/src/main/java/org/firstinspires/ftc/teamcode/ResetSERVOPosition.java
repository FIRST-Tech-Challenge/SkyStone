package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ResetSERVOPosition", group = "")
public class ResetSERVOPosition extends LinearOpMode {

    private Servo GripServo;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        GripServo = hardwareMap.servo.get("GripServo");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                GripServo.setPosition(1);
                // Put loop blocks here.
                telemetry.update();
            }
        }
    }
}