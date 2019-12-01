package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.*;

/**
 * Sensor Calibration TeleOpMode for team Hazmat
 * Displays the RGB Values of Color Sensors and TouchSensor
 */
@TeleOp(name = "HzSensorCalib", group = "Teleop")
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
            //Display RGB Values for hzIntake.detectSkystone
            telemetry.addData("Intake.detectSkystone.Red ", hzIntake.detectSkystone.red() );
            telemetry.addData("Intake.detectSkystone.Green", hzIntake.detectSkystone.green() );
            telemetry.addData("Intake.detectSkystone.Blue", hzIntake.detectSkystone.blue() );
            telemetry.addData("Intake.detectSkystone.Hue", hzIntake.detectSkystone.alpha() );

            //Display RGB Values for hzChassis.leftColorSensor
            telemetry.addData("Chassis.Left.Red ", hzChassis.leftColorSensor.red() );
            telemetry.addData("Chassis.Left.Green", hzChassis.leftColorSensor.green() );
            telemetry.addData("Chassis.Left.Blue", hzChassis.leftColorSensor.blue() );
            telemetry.addData("Chassis.Left.Hue", hzChassis.leftColorSensor.alpha() );

            //Display RGB Values for hzChassis.rightColorSensor
            telemetry.addData("Chassis.Right.Red ", hzChassis.rightColorSensor.red() );
            telemetry.addData("Chassis.Right.Green", hzChassis.rightColorSensor.green() );
            telemetry.addData("Chassis.Right.Blue", hzChassis.rightColorSensor.blue() );
            telemetry.addData("Chassis.Right.Hue", hzChassis.rightColorSensor.alpha() );

            //Display touch sensor pressed or not
            telemetry.addData("Chassis.touch.Pressed", hzChassis.frontleftChassisTouchSensorIsPressed() );

            telemetry.update();

        }
    }

}
