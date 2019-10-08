package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 */
@Autonomous(name= "DepotAuto", group="mecanum autonomous")
@Disabled//comment this out to use
public class DepotAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft     = null; //rear left
    private DcMotor backRight    = null; //rear right
    private DcMotor frontLeft    = null; //front left
    private DcMotor frontRight   = null; //front right
    private DcMotor armMotor     = null; //the arm up and down
    private DcMotor lift         = null; //the actuator latch motor
    private Servo   claw         = null; //the servo on the claw
    private ColorSensor sensorColor = null; //v2 sensor
    private DistanceSensor sensorDistance = null; //v2 sensor

    public static final double OPEN_SERVO  = 0.10; //sets the positions of the servo to max open
    public static final double CLOSE_SERVO = 0.50; //sets the position of the servo to max close
    public static final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;


    //power from -1 to 1 range. positive is forward. negative is backward.
    //move distance in inches
    public void moveDistance(double power, double distance) {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double distancePerRotation = 3.1415 * 6; //pi * diameter (inches)
        double rotations = distance/distancePerRotation; //distance / circumference (inches)
        int encoderDrivingTarget = (int)(rotations*1680);

        if(opModeIsActive()) {
            backLeft.setTargetPosition(encoderDrivingTarget);
            backRight.setTargetPosition(encoderDrivingTarget);
            frontLeft.setTargetPosition(encoderDrivingTarget);
            frontRight.setTargetPosition(encoderDrivingTarget);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            backLeft.setPower(Math.abs(power));//childproof. must have always positive power
            backRight.setPower(Math.abs(power));
            frontLeft.setPower(Math.abs(power));
            frontRight.setPower(Math.abs(power));

            while(backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()) {
                //wait till motor finishes working
                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.update();
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //positive is right, negative is left. inches
    public void strafe(double power, double distance) {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double distancePerRotation = 13.5; //distance per rotations is different than circumference when strafing (inches)
        double rotations = distance/distancePerRotation; //distance / circumference (inches)
        int encoderDrivingTarget = (int)(rotations*1680);

        if(opModeIsActive()) {
            backLeft.setTargetPosition(-encoderDrivingTarget);
            backRight.setTargetPosition(encoderDrivingTarget);
            frontLeft.setTargetPosition(encoderDrivingTarget);
            frontRight.setTargetPosition(-encoderDrivingTarget);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            backLeft.setPower(Math.abs(power));//childproof. must have always positive power
            backRight.setPower(Math.abs(power));
            frontLeft.setPower(Math.abs(power));
            frontRight.setPower(Math.abs(power));

            while(backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()) {
                //wait till motor finishes working
                telemetry.addData("Path", "Strafing "+distance+" inches");
                telemetry.update();
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            telemetry.addData("Strafe", "Complete");
            telemetry.update();

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //positive is right, negative is left
    public void turn(double power, int time) {
        backRight.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        frontLeft.setPower(power);
        sleep(time);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(1000);
    }

    //positive extends, negative retracts,
    public void liftExtend(double power, double distance) {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double distancePerRotation = 0.8
                ; //1 cm up per rotation of motor.
        double rotations = distance/distancePerRotation; //number of rotations needed
        int encoderDrivingTarget = (int)(rotations*1120);//1120 ticks for 40:1 ratio motor

        if(opModeIsActive()) {
            lift.setTargetPosition(encoderDrivingTarget);

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift.setPower(Math.abs(power));//childproof. must always have positive power

            while(lift.isBusy()) {
                //wait till motor finishes working
                telemetry.addData("Lift", "Extending "+distance+" cm");
                telemetry.update();
            }

            lift.setPower(0);

            telemetry.addData("Lift", "Extended");
            telemetry.update();

            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //how long the claw will stay open
    public void drop(int time){
        claw.setPosition(OPEN_SERVO);
        sleep(time);
        claw.setPosition(CLOSE_SERVO);
        sleep(1000);
    }

    public void updateColor() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Hue", hsvValues[0]);
    }

    int i=0;
    //positive distance means left to right and vice versa
    public void sample(double distanceBetweenEachMineral) {
        while(i<3) {
            updateColor();
            if(i<2) {
                if (sensorDistance.getDistance(DistanceUnit.CM) < 20)
                    if (hsvValues[0] > 100) {
                        moveDistance(0.5, -3);
                        strafe(0.5, distanceBetweenEachMineral);
                        moveDistance(0.5, 3);
                    } else {
                        moveDistance(0.5, 5);
                        break;
                    }
                else {
                    moveDistance(0.5, 5);
                    break;
                }
            } else {
                moveDistance(0.5, 5);
                break;
            }
            i++;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        backLeft        = hardwareMap.dcMotor.get("left_drive");
        backRight       = hardwareMap.dcMotor.get("right_drive");
        frontLeft       = hardwareMap.dcMotor.get("front_left");
        frontRight      = hardwareMap.dcMotor.get("front_right");
        armMotor        = hardwareMap.dcMotor.get("arm_motor");
        lift            = hardwareMap.dcMotor.get("lift");
        claw            = hardwareMap.servo.get("claw");
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "v2sensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "v2sensor");

        claw.setPosition(CLOSE_SERVO);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        //strafe(0.5, 13.5);
        //liftExtend(1, 15.5); //lower the robot. extend lift 15.5 cm
        //strafe(0.3, -5.25); //unhook by strafing left 5.25 inches
        //moveDistance(0.5, 4);
        //strafe(0.3, 5.25);
        //moveDistance(0.5, 4);
        //strafe(0.3, -10);
        //moveDistance(0.5, 3);
        sample(14); //left to right
        //if(i==0)
        //    turn(0.5, 300);
        //else if(i==2)
        //    turn(-0.5, 300);
        //moveDistance(0.5, 10);
        //drop(1000);
        //40 inches

    }
}