package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotlib.motor.CalibratingMotor;

@Disabled
@Autonomous(name="Calibration Test", group="Test")
public class CalibrationTest extends LinearOpMode
{
    private DcMotor testMotor;
    private CalibratingMotor testCalibrator;
    private DigitalChannel touchSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");

        // Init motor
        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init sensor
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        // Init calibrator
        testCalibrator = new CalibratingMotor(testMotor, touchSensor);

        telemetry.addData("Objets initialized", "waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            testCalibrator.calibrateToZero(DcMotorSimple.Direction.REVERSE, 0.5);

            telemetry.addData("MotorPos", testMotor.getCurrentPosition());
            telemetry.addData("MotorDirection", testMotor.getDirection());
            telemetry.addData("MotorPower", testMotor.getPower());
            telemetry.addData("MotorMode", testMotor.getMode());
            telemetry.addData("TouchSensor", touchSensor.getState());
            telemetry.update();
        }
    }
}
