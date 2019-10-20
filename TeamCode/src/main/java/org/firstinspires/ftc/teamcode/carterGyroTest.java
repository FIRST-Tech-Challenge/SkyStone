package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import static java.lang.Thread.sleep;

@TeleOp(name = "REDWOOD", group = "REDWOOD")
public class carterGyroTest extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    public DcMotor armRaiser;
    public DcMotor armExtended;
    GyroSensor gyro;

    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    @Override
    public void init() {
       /* TL = hardwareMap.get(DcMotor.class, "TL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BR = hardwareMap.get(DcMotor.class, "BR");*/
        armRaiser = hardwareMap.get(DcMotor.class, "armRaiser");
        armExtended = hardwareMap.get(DcMotor.class, "armExtender");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro=hardwareMap.get(GyroSensor.class,"gyro");
        gyro.resetZAxisIntegrator();
        runtime.reset();
/*        do {
            gyro.calibrate();
        }
        while (gyro.isCalibrating());*/
        gyro.calibrate();
    }
    int armLevel = 0;
    double[] armAngles = {0,};
    double[] armDistance = {16, 22, };

    double rawz = 0;
    double rawx = 0;
    double rawy = 0;

    @Override
    public void loop() {
        /*TL = hardwareMap.get(DcMotor.class, "TL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BR = hardwareMap.get(DcMotor.class, "BR");*/
        armRaiser = hardwareMap.get(DcMotor.class, "armRaiser");
        armExtended = hardwareMap.get(DcMotor.class, "armExtender");


        armRaiser.setPower(-gamepad1.right_stick_y);
        armExtended.setPower(-gamepad1.left_stick_y);
        extension(armExtended.getCurrentPosition(), 700);

        if (gamepad1.dpad_up) {
            armLevel++;
        }
        if (gamepad1.dpad_down) {
            armLevel--;
        }

        if (gamepad1.b) {
            rawz = gyro.getHeading();
        }
        telemetry.addData("Head", rawz);
        telemetry.addData("Encoder Ext", armExtended.getCurrentPosition());
        telemetry.addData("Encoder Raise", armRaiser.getCurrentPosition());
        telemetry.addData("Arm Level:", armLevel + 1);
        if (gamepad1.right_bumper) {
            armExtended.setPower(1);
            try
            {
                Thread.sleep(1000);
            }
            catch(InterruptedException ex)
            {
                Thread.currentThread().interrupt();
            }
            armExtended.setPower(0);
        }
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
    }

    public double e() {
        double Range = 0.6096;
        double conversionFactor = 0.0163624617; //meters over encoder
        double angle = (gyro.getHeading())*(3.14/180);
        double extension = (Range/(Math.cos(angle)));
        //return (int)((extension* (1/conversionFactor)));
        return (angle);
    }

    public void extension(int currentPos, int encoder) {
        while(((currentPos<(encoder-10)) ||  (currentPos>(encoder+10))) && gamepad1.a) {
            armExtended.setPower(0.2);
        }
    }

    public void arm(int armLevel) {

    }

}

