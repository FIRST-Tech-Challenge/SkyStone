package org.firstinspires.ftc.teamcode.gamecode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * Created by Alec Krawciw on 2017-08-23.
 */

public class Awesomness extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {
        Robot ceaser = new Robot();
        ColorSensor sensor = RC.h.colorSensor.get("color");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        BNO055IMU imu = RC.h.get(BNO055IMU.class, "adafruit");
        imu.initialize(params);

        sensor.enableLed(false);
        ceaser.motorL.setReverse(true);
        ceaser.motorR.setReverse(false);
        waitForStart();

        while (sensor.red() < 100 && opModeIsActive()) {
            ceaser.forward(0.15);
        }
        ceaser.backward(0.5);
        sleep(500);
        imuTurnL(ceaser, imu, 90, 0.5);
        sleep(1000);
        ceaser.stop();


    }

    public void imuTurnL(Robot r, BNO055IMU imu, double degrees, double speed) {

        r.turnL(speed);
        double beginAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle(imu)[0]);
        double targetAngle = MathUtils.cvtAngleToNewDomain(beginAngle - degrees);

        Log.i("targetAngleIMUTURNL", targetAngle + "");

        while (RC.l.opModeIsActive()) {

            double currentAngle = MathUtils.cvtAngleToNewDomain(getIMUAngle(imu)[0]);
            double angleToTurn = MathUtils.cvtAngleJumpToNewDomain(currentAngle - targetAngle);

            Log.i("CurrentAngleXAIMUTURNL", currentAngle + "");
            Log.i("AngleToTurnIMUTURNL", angleToTurn + "");

            r.turnL(Math.signum(angleToTurn) * (Math.abs(angleToTurn) / 180 * speed));

            if (Math.abs(angleToTurn) < 3) {
                break;
            }//if
        }//while

        r.stop();
    }//imuTurnL

    public double[] getIMUAngle(BNO055IMU imu) {
        Orientation orient = imu.getAngularOrientation();

        return new double[]{-orient.firstAngle, -orient.secondAngle, -orient.thirdAngle};
    }//getIMUAngle
}

