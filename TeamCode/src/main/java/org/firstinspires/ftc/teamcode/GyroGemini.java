package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

@TeleOp(name = "REDWOOD", group = "REDWOOD")
public class GyroGemini extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    public DcMotor armRaiser;
    public DcMotor armExtended;
    GyroSensor gyro;
   // public GyroSensor gyro;
    //public ModernRoboticsI2cGyro gyro;

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
        //gyro=hardwareMap.get(GyroSensor.class,"gyro");

        runtime.reset();
/*        do {
            gyro.calibrate();
        }
        while (gyro.isCalibrating());*/
        gyro.calibrate();
    }

    @Override
    public void loop() {
        /*TL = hardwareMap.get(DcMotor.class, "TL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BR = hardwareMap.get(DcMotor.class, "BR");*/
        armRaiser = hardwareMap.get(DcMotor.class, "armRaiser");
        armExtended = hardwareMap.get(DcMotor.class, "armExtender");
        gyro = hardwareMap.gyroSensor.get("gyro");

        armRaiser.setPower(-gamepad1.right_stick_y);
        extension(armExtended.getCurrentPosition(), 700);

        telemetry.addData("Process", e());
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
    }

    public int e() {
        double Range = 0.6096;
        double conversionFactor = 0.0163624617; //meters over encoder
        double angle = (gyro.getHeading())*(3.14/180);
        double extension = (Range/(Math.cos(angle)));
        return (int)((extension* (1/conversionFactor)));
    }

    public void extension(int currentPos, int encoder) {
        while(((currentPos<(encoder-10)) ||  (currentPos>(encoder+10))) && gamepad1.a) {
            armExtended.setPower(0.2);
        }
    }
}

