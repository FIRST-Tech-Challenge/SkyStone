package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroBot extends CameraBot {

    BNO055IMU imu;
    double startAngle, power = 0.15;


    public GyroBot(LinearOpMode opMode) {
        super(opMode);
    }


    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";


        imu =  hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void resetAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        RobotLog.d(String.format("Reset Angle : %.3f , %.3f, %.3f", angles.firstAngle, angles.secondAngle, angles.thirdAngle));
        startAngle = angles.firstAngle;
    }


    public double getDeltaAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - startAngle;
        RobotLog.d(String.format("Delta Angle : %.3f from %.3f, %.3f, %.3f", deltaAngle, angles.firstAngle, angles.secondAngle, angles.thirdAngle));

        return deltaAngle;
    }


    public void goBacktoStartAngle() {



        int direction ;
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double delta = getDeltaAngle();

        while (Math.abs(delta) > 2){
            if (delta < 0){
                // turn clockwize
                direction = -1;
            }
            else{
                // turn CC wize
                direction = 1;
            }
            leftFront.setPower(-power * direction);
            rightFront.setPower(power * direction);
            leftRear.setPower(-power * direction);
            rightRear.setPower(power * direction);

            delta = getDeltaAngle();

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }
}