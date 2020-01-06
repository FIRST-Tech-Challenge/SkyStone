package org.firstinspires.ftc.teamcode.UnitTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.*;

import java.util.Locale;

/**
 * Sensor Calibration TeleOpMode for team Hazmat
 * Displays the RGB Values of Color Sensors and TouchSensor
 */
@Disabled
@TeleOp(name = "HzSensorCalib", group = "TeleopTest")
public class SensorCaliberation extends LinearOpMode{

    HzGamepad1 hzGamepad1;
    Chassis hzChassis;
    Arm hzArm;
    Intake hzIntake;

    @Override
    public void runOpMode() {
        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        hzChassis = new Chassis(hardwareMap);
        hzArm = new Arm(hardwareMap);
        hzIntake = new Intake(hardwareMap);
        hzGamepad1 = new HzGamepad1(gamepad1);

        //Initialize Subsystems - Chassis, Arm, Intake.
        hzChassis.initChassis();
        hzArm.initArm();
        hzIntake.initIntake();

        telemetry.setAutoClear(true);
        telemetry.addData("Init SensorCalib", "v:1.0");

        //Wait for pressing Run on controller
        waitForStart();

        //Run Robot to display color values and touch sensor inputs
        while (opModeIsActive()) {
            //Display RGB Values for hzIntake.detectSkystoneColor
            // telemetry.addData("Intake.detectSkystoneColor.Red ", hzIntake.detectSkystoneColor.red() );
            // telemetry.addData("Intake.detectSkystoneColor.Green", hzIntake.detectSkystoneColor.green() );
            // telemetry.addData("Intake.detectSkystoneColor.Blue", hzIntake.detectSkystoneColor.blue() );
            // telemetry.addData("Intake.detectSkystoneColor.Alpha", hzIntake.detectSkystoneColor.alpha() );
            // telemetry.addData("Intake.detectSkystonedistance.Distance", String.format(Locale.US, "%.02f", hzIntake.detectSkystoneDistance.getDistance(DistanceUnit.INCH)));
            // telemetry.addData("Intake.detectSkystonedetected", hzIntake.stoneDetected);
            // telemetry.addData("Intake.detectSkystonedetected", hzIntake.skystoneDetected);

            //Display RGB Values for hzChassis.leftColorSensor
            telemetry.addData("Chassis.Left.Red ", hzChassis.leftColorSensor.red() );
            telemetry.addData("Chassis.Left.Green", hzChassis.leftColorSensor.green() );
            telemetry.addData("Chassis.Left.Blue", hzChassis.leftColorSensor.blue() );
            telemetry.addData("Chassis.Left.Alpha", hzChassis.leftColorSensor.alpha() );
            telemetry.addData("Chassis.Left.Hue", hzChassis.leftColorSensor.argb() );

            //Display RGB Values for hzChassis.rightColorSensor
            telemetry.addData("Chassis.Right.Red ", hzChassis.rightColorSensor.red() );
            telemetry.addData("Chassis.Right.Green", hzChassis.rightColorSensor.green() );
            telemetry.addData("Chassis.Right.Blue", hzChassis.rightColorSensor.blue() );
            telemetry.addData("Chassis.Right.Alpha", hzChassis.rightColorSensor.alpha() );
            telemetry.addData("Chassis.Left.Hue", hzChassis.leftColorSensor.argb() );

            //Display touch sensor pressed or not
            telemetry.addData("Chassis.touch.Pressed", hzChassis.frontleftChassisTouchSensorIsPressed() );

            telemetry.update();

        }
    }

}
