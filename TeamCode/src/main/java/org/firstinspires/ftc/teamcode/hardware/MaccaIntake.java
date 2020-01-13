package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MaccaIntake {

    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    private DcMotorEx intake_left, intake_right;

    public MaccaIntake(OpMode parentOpMode) {
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
    }

    public void initializeIntake() {
        parentOpMode.telemetry.addLine("MaccaIntake Initializing...");
        // Find intake motors
        intake_left = hardwareMap.get(DcMotorEx.class,"intake_left");
        intake_right = hardwareMap.get(DcMotorEx.class,"intake_right");
        // Encoders are not present, so encoders should be ignored
        intake_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // So that both intake wheels run in the same direction
        intake_left.setDirection(DcMotorSimple.Direction.REVERSE);
        // So that the intake properly stops
        intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        parentOpMode.telemetry.addLine("Intake Initialization complete.");
    }

    public void runIntake(double power) {
        intake_left.setPower(power);
        intake_right.setPower(power);
    }

}
