package org.firstinspires.ftc.teamcode.robotModels;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MechanumFourWheel {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    double calibFL = 1.00f;
    double calibFR = 1.00f;
    double calibBL = 1.00f;
    double calibBR = 1.00f;

    DriveMode mode;

    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

    static enum DriveMode {
        RAW_POWER,
        ENCODER,
        TELE_OP_THREE_DIM,
        TELE_OP_STICK_AVERAGE
    }

    public MechanumFourWheel(DriveMode mode, DcMotor[] motors, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.mode = mode;
        motorFL = motors[0];
        motorFR = motors[1];
        motorBL = motors[2];
        motorBR = motors[3];
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void setupMotors() {
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        if (mode.equals(DriveMode.ENCODER)) {
            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void correctDirections(DcMotorSimple.Direction[] directions) {
        motorFL.setDirection(directions[0]);
        motorFR.setDirection(directions[1]);
        motorBL.setDirection(directions[2]);
        motorBR.setDirection(directions[3]);
    }

    public void calibrateMotors(float[] calibs) {
        double calibFL = calibs[0];
        double calibFR = calibs[1];
        double calibBL = calibs[2];
        double calibBR = calibs[3];
    }

    public void doAction() {

    }

}
