package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class LibraryUltrasonicTurn {

    HardwareBeep robot = new HardwareBeep();
    //SensorMB1242 sonic = robot.ultrasonic;
    Telemetry telemetry;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    double angle_variable;
    double speed;
    boolean aButton, bButton, touched;

    long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;


    /**
     * The hardware class needs to be initialized before this function is called
     */
    public void init(HardwareBeep myRobot, Telemetry myTelemetry) {
        robot = myRobot;
        telemetry = myTelemetry;

    }

    public void ComputePID() {
        long now = System.currentTimeMillis();
        double timeChange = (double) (now - lastTime);
        double error = Setpoint - Input;
        errSum += (error * timeChange);
        double dErr = (error - lastErr);

        Output = kp * error + ki * errSum + kd * dErr;
        lastErr = error;
        lastTime = now;

    }

    public void SetTunings(double Kp, double Ki, double Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }


    public double turnUltrasonic(float targetHeading) {
        int original_anglez = 0;
//        BNO055IMU imu;
        int xVal, yVal, zVal = 0;
        int heading = 0;
        int angleZ = 0;
        float MIDPOWER = 0;
        double DRIVEGAIN = 1;
        double TOLERANCE = .5;
        int timer = 0;
        double currentHeading, headingError, driveSteering, leftPower, rightPower, oldCurrentHeading = 0.0;
        long startTime = 0;
        double polarity = 1;
        polarity = targetHeading > 0 ? 1 : -1;

        telemetry.addData("Distance read", robot.leftSonic.getDistance());
        telemetry.addData("Distance read", robot.rightSonic.getDistance());
        telemetry.update();
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        telemetry.addData("Current Pos", currentHeading);
//        updateTelemetry(telemetry);

        startTime = System.currentTimeMillis();
//        currentHeading = robot.sonic.getDistance();
        SetTunings(.02, 0, 0.1);

        Setpoint = targetHeading;
        Input = robot.leftSonic.getDistance();
        Input = robot.rightSonic.getDistance();
//        telemetry.addData("Current Pos ", currentHeading);
        telemetry.addData("Setpoint ", Setpoint);
        telemetry.addData("Input ", Input);
        telemetry.update();
        //        sleep(5000);

        //Input = currentHeading;

        Output *= polarity;

        do {

            ComputePID();
            robot.leftFront.setPower(Output);
            robot.leftBack.setPower(Output);
            robot.rightFront.setPower(-Output);
            robot.rightBack.setPower(-Output);
            timer++;
            //sleep(1000);
            Input = robot.leftSonic.getDistance();
            Input = robot.rightSonic.getDistance();
            //sleep(1000);
            telemetry.addData("curHeading", Input);
            telemetry.addData("tarHeading", Setpoint);
            telemetry.update();
            //} while (Input < targetHeading && (System.currentTimeMillis() < (startTime + 6000)));
        }
        while ((Math.abs(Input - Setpoint) > TOLERANCE) || (System.currentTimeMillis() < (startTime + 1050)));


        telemetry.addData("curHeading", Input);
        telemetry.addData("tarHeading", Setpoint);
        telemetry.addData("leftPwr", -Output);
        telemetry.addData("rightPwr", Output);
        //telemetry.addData("headingErr", headingError);
        //telemetry.addData("driveSteer", driveSteering);
        telemetry.addData("DRIVEGAIN", DRIVEGAIN);
        telemetry.update();

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        return Input;

    }

}