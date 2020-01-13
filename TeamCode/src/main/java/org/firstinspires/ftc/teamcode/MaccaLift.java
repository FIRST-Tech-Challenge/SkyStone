package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class MaccaLift {

    public static double lift_kP = 0.004;

    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    private DcMotorEx lift_left, lift_right;
    private CRServo rack;
    private Servo chad;
    private double liftTargetPos;

    public MaccaLift(OpMode parentOpMode) {
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
    }

    public void initializeLift() {
        parentOpMode.telemetry.addLine("MaccaLift Initializing...");
        // map hardware
        lift_left = hardwareMap.get(DcMotorEx.class,"lift_left");
        lift_right = hardwareMap.get(DcMotorEx.class,"lift_right");
        rack = hardwareMap.crservo.get("bob");
        chad = hardwareMap.servo.get("moveChad");
        // make motors compatible
        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);
        // reset and deactivate encoders
        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set motors to stop when stopped
        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // tell the world you've succeeded
        parentOpMode.telemetry.addLine("Lift Initialization complete.");
    }

    public void runLift(double rate){
        liftTargetPos += 25*rate;
        liftTargetPos -= 25*rate;
        // Limit clauses
        if (liftTargetPos >= 2020) {
            liftTargetPos = 2020;
        } else if (liftTargetPos <= 0) {
            liftTargetPos = 0;
        }
        // Process the value
        lift_backend(liftTargetPos, 0.7, 0.004);
    }

    public void lift_backend(double desiredPosition, double maxSpeed, double kP) {
        double error = desiredPosition - getLiftPosition();
        lift_right.setPower(kP*error);
        lift_left.setPower(kP*error);
    }

    public void runRack(double rate) {
        rack.setPower(rate);
    }

    public void runChad(boolean open, boolean closed) {
        if (open && closed) {
            chad.setPosition(chad.getPosition());
        } else if (closed) {
            chad.setPosition(0);
        } else if (open) {
            chad.setPosition(1);
        }
    }

    public double getChadPosition() { return chad.getPosition(); }
    public int getLiftPosition() { return lift_right.getCurrentPosition(); }
    public int getLiftTargetPosition() { return (int) liftTargetPos; }

}
