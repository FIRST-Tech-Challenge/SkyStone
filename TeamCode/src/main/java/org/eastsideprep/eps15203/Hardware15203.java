package org.eastsideprep.eps15203;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Hardware definitions for 15203
 */
public class Hardware15203 {
    /* Public OpMode members. */
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor [] allMotors;
    public DcMotor zArmMotor = null;
    public Servo grabberServo = null;
    public Gyroscope gyro = null;

    double [] rotationArray;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware15203() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        leftFrontMotor = hwMap.dcMotor.get("LF");
        rightFrontMotor = hwMap.dcMotor.get("RF");
        leftBackMotor = hwMap.dcMotor.get("LB");
        rightBackMotor = hwMap.dcMotor.get("RB");

        zArmMotor = hwMap.dcMotor.get("ZM");
        zArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberServo = hwMap.servo.get("GS");

        allMotors = new DcMotor[]{ leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor};
        rotationArray= new double[]{-1.0, 1.0, -1.0, 1.0};

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);


         for (DcMotor m : allMotors) {
            m.setPower(0.0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // really? good for autonomous. But in driver control?

        }
    }



    public double[] getDrivePowersFromAngle(double angle) {
        double[] unscaledPowers = new double[4];
        unscaledPowers[0] = Math.sin(angle + Math.PI / 4);
        unscaledPowers[1] = Math.cos(angle + Math.PI / 4);
        unscaledPowers[2] = unscaledPowers[1];
        unscaledPowers[3] = unscaledPowers[0];
        return unscaledPowers;
    }

    void threadSleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            //do nothing
        }
    }

    /*
                VARIOUS FUNCTIONS EVEREST WROTE FOR AUTONOMOUS ROUTINES:
     - allDrive: sets all motors to a given power, condensing code needed to drive.
     - turn: turns in place at a given power for a given number of
        milliseconds. There is no way to input degrees.
     - spinTurn: a project of Ben's that is still in progress. When finished, it will
        allow the robot to turn on a pivot instead of in place.
     - garageLift and garagePlace: turns the CR servos on the garage mechanism in
        the directions needed to lift or release a block for a given number of milliseconds.
     */

    public void allDrive(double power, int milliseconds){
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);

        threadSleep(milliseconds);

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public void turn(double power, int milliseconds){
        //Front motors
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        //Back motors
        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(power);

        threadSleep(milliseconds);

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public void spinTurn(double power, int milliseconds){
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        //Back motors
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);

        threadSleep(milliseconds);

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        //Back motors
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }




}

