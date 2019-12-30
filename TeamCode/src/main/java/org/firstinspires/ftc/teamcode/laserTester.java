package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "LASER")
public class laserTester extends LinearOpMode {
    public double position;

    @Override
    public void runOpMode() throws InterruptedException {
        TypexChart chart = new TypexChart();
        CONSTANTS constants = new CONSTANTS();
        chart.init(hardwareMap);

        while(opModeIsActive()&& !chart.imu.isGyroCalibrated()){
            telemetry.addData("Calibration State: ", chart.imu.isGyroCalibrated());
            telemetry.update();
        }

        telemetry.addData("Waiting", "...");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Laser Reading: ", chart.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("POSITION: ", position);
            telemetry.addData("speedMultip", constants.speedMultip);
            telemetry.update();

            if(gamepad1.right_stick_button){
                constants.setPositionToggle(!constants.isPositionToggle());
            }

            if (gamepad1.left_stick_button){
                constants.setSpeedToggle(!constants.isSpeedToggle());
            }

            if(constants.isSpeedToggle() == true){
                constants.speedMultip = 0.5;
            }
            else {
                constants.speedMultip = 1;
            }

            if (constants.isPositionToggle() == true){
                position = constants.CLOSEPOSITION;
            }
            else position = constants.OPENPOSITION;

            chart.TL.setPower(gamepad1.left_stick_y/constants.speedMultip);
            chart.BL.setPower(gamepad1.left_stick_y/constants.speedMultip);
            chart.TR.setPower(gamepad1.right_stick_y/constants.speedMultip);
            chart.BR.setPower(gamepad1.right_stick_y/constants.speedMultip);


        }
    }
}
