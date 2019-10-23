package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "OverallOPMode", group = "")
public class OverallOPMode extends LinearOpMode {

    private DcMotor motorDriveBackLeft;
    private DcMotor motorDriveBackRight;
    private DcMotor motorDriveFrontLeft;
    private DcMotor motorDriveFrontRight;
    private DcMotor motorDriveLifter;
    private Servo gripServo;
    private Servo trayDragServo;

    @Override
    public void runOpMode () {
        motorDriveBackLeft = hardwareMap.dcMotor.get("motorDriveBackLeft")
    }
}
