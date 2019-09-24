//Run from the package
package org.firstinspires.ftc.teamcode.subsystems;

//Import necessary items

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.imu.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.filters.LeviColorFilter;
//
//import org.opencv.core.Point;

@Disabled
public class DriveFunctions extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    BNO055IMU boschIMU;

    IIMU imu;

//    //TFOD Variables
//    private static final String VUFORIA_KEY = "Adp/KFX/////AAAAGYMHgTasR0y/o1XMGBLR4bwahfNzuw2DQMMYq7vh4UvYHleflzPtt5rN2kFp7NCyO6Ikkqhj/20qTYc9ex+340/hvC49r4mphdmd6lI/Ip64CbMTB8Vo53jBHlGMkGr0xq/+C0SKL1hRXj5EkXtSe6q9F9T/nAIcg9Jr+OfAcifXPH9UJYG8WmbLlvpqN+QuVA5KQ6ve1USpxYhcimV9xWCBrq5hFk1hGLbeveHrKDG3wYRdwBeYv3Yo5qYTsotfB4CgJT9CX/fDR/0JUL7tE29d1v1eEF/VXCgQP4EPUoDNBtNE6jpKJhtQ8HJ2KjmJnW55f9OqNc6SsULV3bkQ52PY+lPLt1y4muyMrixCT7Lu";
//    private VuforiaLocalizer vuforia;
//    private TFObjectDetector tfod;
//    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
//    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
//    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    /**
     * Initialize all the hardware
     * This creates a data type DriveFunctions to store all the hardware devices
     */
    public DriveFunctions(DcMotor.ZeroPowerBehavior type, DcMotor leftMotorFront, DcMotor rightMotorFront, DcMotor leftMotorBack, DcMotor rightMotorBack, BNO055IMU boschIMU)
    {


        //These lines enable us to store the motors, sensors and CDI without having to write them over and over again
        //Initialize DC motors
        this.leftMotorFront = leftMotorFront;
        this.leftMotorBack = leftMotorBack;
        this.rightMotorFront = rightMotorFront;
        this.rightMotorBack = rightMotorBack;

        this.boschIMU = boschIMU;

        //Reverse some motors and keep others forward
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set the drive motors to brake mode to prevent rolling due to chain
        leftMotorFront.setZeroPowerBehavior(type);
        leftMotorBack.setZeroPowerBehavior(type);
        rightMotorFront.setZeroPowerBehavior(type);
        rightMotorBack.setZeroPowerBehavior(type);

        imu = new BoschIMU(boschIMU);
        imu.initialize();

    }

    /**
     * Set sensor addresses, modes and DC motor directions, modes
     */
    public void initializeRobotFloat()
    {
//        //Set the sensor to the mode that we want
//        colorSensorCenter.enableLed(true);
//        colorSensorRight.enableLed(true);

        //Reverse some motors and keep others forward
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set the drive motors to float so it coasts. Puts less strain on motors.
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void initializeRobotBrake()
    {
//        //Set the sensor to the mode that we want
//        colorSensorCenter.enableLed(true);
//        colorSensorRight.enableLed(true);

        //Set the drive motors to brake mode to prevent rolling due to chain
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Takes in motor powers for 4 drive motors
     */
    public void setDriveMotorPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        leftMotorFront.setPower((float)leftFrontPower);
        leftMotorBack.setPower((float)leftBackPower);
        rightMotorFront.setPower((float)rightFrontPower);
        rightMotorBack.setPower((float)rightBackPower);
    }

    /**
     * If this function is called, stop the drive motors
     */
    public void stopDriving()
    {
        //Set all drive motor powers as zero
        setDriveMotorPowers( 0.0,0.0,0.0,0.0);
    }


    /**
     * If this function is called, turn on the drive motors at the given powers to make it drive forward or backwards
     */
    public void driveTeleop(double power)
    {
        //Send all the motors in the same direction
        setDriveMotorPowers(power, power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn left
     */
    public void leftTurnTeleop(double power)
    {
        //Turn the left motors backwards and the right motors forward so that it turns left
        setDriveMotorPowers(power, power, -power, -power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn right
     */
    public void rightTurnTeleop(double power)
    {
        //Turn the right motors backwards and the left motors forward so that it turns right
        setDriveMotorPowers(-power, -power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the
     * given powers, to make it shift in the desired direction
     */
    public void shiftTeleop(double power)
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift
        setDriveMotorPowers(-power, power, power, -power);
    }

    public void coeffShiftTeleop(double power)
    {
        double leftPower = 0.8 * power;
        double rightPower = power;
        setDriveMotorPowers(-leftPower, leftPower, rightPower, -rightPower);
    }

    public void resetEncoders()
    {
        //Reset the encoders
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Use the encoders
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * Takes in powers for 4 drive motors, as well as 4 encoder distances
     * Allows us to run at the entered power, for the entered distance
     */
    public void moveDriveMotorsWithEncoders(int leftFrontDegrees, int leftBackDegrees, int rightFrontDegrees, int rightBackDegrees, double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower)
    {
        //Reset the encoders
        resetEncoders();

        //Set up the motors to run to the given position
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the target position as the corresponding values entered
        leftMotorFront.setTargetPosition(leftFrontDegrees);
        leftMotorBack.setTargetPosition(leftBackDegrees);
        rightMotorFront.setTargetPosition(rightFrontDegrees);
        rightMotorBack.setTargetPosition(rightBackDegrees);

        //Turn on the motors at the corresponding powers
        setDriveMotorPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

        //Empty while loop while the motors are moving
        while ((leftMotorFront.isBusy()) && (rightMotorFront.isBusy()) && (leftMotorBack.isBusy()) && (rightMotorBack.isBusy()))
        { }

        //Stop driving
        stopDriving();

        //Use the encoders in the future
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drive for the given distance at the given power
     * @param degrees distance
     */
    public void driveAutonomous(double power, int degrees) throws InterruptedException
    {
        //Everything in the same direction creates linear driving
        moveDriveMotorsWithEncoders(-degrees, -degrees, -degrees, -degrees, -power, -power, -power, -power);
        stopDriving();
        Thread.sleep(10);
        stopDriving();
    }

    /**
     * Turn left for the given distance at the given power
     * @param degrees distance
     */
    public void leftTurnAutonomous(float power, int degrees) throws InterruptedException
    {
        //Left motors backwards and right motors forwards gives us a left turn
        moveDriveMotorsWithEncoders(degrees, degrees, -degrees, -degrees, power, power, -power, -power);
        stopDriving();
        Thread.sleep(10);
        stopDriving();
    }

    public void encoderCoeffLeftTurn(double power, int degrees) throws InterruptedException
    {
        double factor = 21.7;
        int newDegrees = (int) factor * degrees;
        leftTurnAutonomous((float) power, newDegrees);
    }

    /**
     * Turn right for the given distance at the given power
     * @param degrees distance
     */
    public void rightTurnAutonomous(float power, int degrees) throws InterruptedException
    {
        //Right motors backwards and left motors forwards gives us a right turn
        moveDriveMotorsWithEncoders(-degrees, -degrees, degrees, degrees, -power, -power, power, power);
        stopDriving();
        Thread.sleep(10);
        stopDriving();
    }

    public void encoderCoeffRightTurn(float power, int degrees) throws InterruptedException
    {
        double factor = 21.7;
        int newDegrees = (int) factor * degrees;
        rightTurnAutonomous(power, newDegrees);
    }


    public void rightTurnIMU(double power, int target) throws InterruptedException
    {
        while(boschIMU.getAngularOrientation().firstAngle > target)
        {
            rightTurnTeleop(power);
        }
        stopDriving();
        while (boschIMU.getAngularOrientation().firstAngle < target)
        {
            leftTurnTeleop(0.2);
        }
        stopDriving();
    }

    public void  leftTurnIMU(double power, int target) throws InterruptedException
    {
        while(boschIMU.getAngularOrientation().firstAngle < target)
        {
            leftTurnTeleop(power);
        }

        stopDriving();
        while (boschIMU.getAngularOrientation().firstAngle > target)
        {
            rightTurnTeleop(0.2);
        }
        stopDriving();

    }

    public void pidIMULeft(float power, int degrees)
    {
        while (Math.abs((double) degrees - boschIMU.getAngularOrientation().firstAngle) > 1)
        {
            while (boschIMU.getAngularOrientation().firstAngle < degrees)
            {
                leftTurnTeleop(power);
            }
            stopDriving();
            while (boschIMU.getAngularOrientation().firstAngle > degrees)
            {
                rightTurnTeleop(power);
            }
            stopDriving();
        }
        stopDriving();
    }

    public void pidIMURight(float power, int degrees)
    {
        while (Math.abs((double) degrees - boschIMU.getAngularOrientation().firstAngle) > 1)
        {
            while (boschIMU.getAngularOrientation().firstAngle > degrees)
            {
                rightTurnTeleop(power);
            }
            stopDriving();
            while (boschIMU.getAngularOrientation().firstAngle < degrees)
            {
                leftTurnTeleop(power);
            }
            stopDriving();
        }
        stopDriving();
    }


    /**
     * Shift left for the given distance at the given power
     * @param degrees distance
     */
    public void leftShiftAutonomous(double power, int degrees) throws InterruptedException
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift left
        moveDriveMotorsWithEncoders(degrees, -degrees, -degrees, degrees, power, -power, -power, power);
        stopDriving();
        Thread.sleep(10);
        stopDriving();
    }

    /**
     * Shift right for the given distance at the given power
     * @param degrees distance
     */
    public void rightShiftAutonomous(float power, int degrees) throws InterruptedException
    {
        //This sequence of forwards, backwards, backwards, forwards makes the robot shift right
        moveDriveMotorsWithEncoders(-degrees, degrees, degrees, -degrees, -power, power, power, -power);
        stopDriving();
        Thread.sleep(10);
        stopDriving();
    }

    /**
     * @param colorSensor take in the correct color sensor
     * @return returns true if the supplied ColorSensor either red or blue.  False otherwise
     */
    public boolean iSeeAColor(ColorSensor colorSensor)
    {
        //This is an array that stores the hue[0], saturation[1], and value[2], values
        float[] hsvValues = {0F, 0F, 0F};

        //Convert from RGB to HSV (red-green-blue to hue-saturation-value)
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        //If no value, return false
        if (hsvValues[2] == 0)
        {
            return false;
        }

        //Otherwise return true
        return true;
    }

    /**
     * Determines what color the color sensor is seeing
     * @param colorSensor take in the correct color sensor
     * @return The string "Blue" if we see the color blue, "Red" if we see the color red
     */
    public Boolean isYellow(ColorSensor colorSensor)
    {
        //Define float for hue
        float alpha = colorSensor.alpha();


        //If hue is greater than 120, we are looking at yellow so return yellow
        if (alpha > 100) //SOMETHING
        {
            return true;
        }

        //Otherwise return not yellow
        return false;
    }

    public void chassisTeleOp(Gamepad gamepad1, Gamepad gamepad2)
    {
        float drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.6);
        float shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.6);
        float leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.6);
        float rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.6);

        //Drive if joystick pushed more Y than X on gamepad1 (fast)
        if (Math.abs(drivePower) > Math.abs(shiftPower))
        {
            driveTeleop(drivePower);
        }

        //Shift if pushed more on X than Y on gamepad1 (fast)
        if (Math.abs(shiftPower) > Math.abs(drivePower))
        {
            shiftTeleop(shiftPower);
        }

        //If the left trigger is pushed on gamepad1, turn left at that power (fast)
        if (leftTurnPower > 0)
        {
            leftTurnTeleop(leftTurnPower);
        }

        //If the right trigger is pushed on gamepad1, turn right at that power (fast)
        if (rightTurnPower > 0)
        {
            rightTurnTeleop(rightTurnPower);
        }

        //If the joysticks are not pushed significantly shut off the wheels
        if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
        {
            stopDriving();
        }
    }

    public void spoolInFully(DcMotor mineralSpool, ColorSensor colorSensor, Gamepad gamepad1, Gamepad gamepad2)
    {
        while (!iSeeAColor(colorSensor))
        {
            chassisTeleOp(gamepad1, gamepad2);
            mineralSpool.setPower(-1.0);
        }
        stopDriving();
        while (!isYellow(colorSensor))
        {
            chassisTeleOp(gamepad1, gamepad2);
            mineralSpool.setPower(-1.0);
        }
        stopDriving();
        mineralSpool.setPower(0.0);
    }

    /**
     * If this function is called, it enables us to run one DC motor to a specific distance
     */
    public static void oneMotorEncoder(DcMotor motor, double power, int degrees) throws InterruptedException
    {
        int firstPos, secondPos;

        //Use the encoder
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up the motor to run to the given position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the target position as the value entered
        motor.setTargetPosition(motor.getCurrentPosition() + degrees);

        //Turn the motor on at the corresponding power
        motor.setPower((float) power);

        //Empty while loop while the motor is moving
        while ((motor.isBusy()))
        {
            firstPos = motor.getCurrentPosition();
            Thread.sleep(75);
            secondPos = motor.getCurrentPosition();

            if (Math.abs(firstPos - secondPos) < 5)
            {
                break;
            }
        }

        //Stop the motor
        motor.setPower(0.0);

        //Use the encoder in the future
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void omeWithDriveMotors(DcMotor motor, double power, int degrees, Gamepad gamepad1, Gamepad gamepad2)
    {
        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runTime.reset();

        //Use the encoder
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up the motor to run to the given position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the target position as the value entered
        motor.setTargetPosition(motor.getCurrentPosition() + degrees);

        //Turn the motor on at the corresponding power
        motor.setPower((float)power);

        //Empty while loop while the motor is moving
        while ((motor.isBusy()) && runTime.time() < 3000)
        {
            chassisTeleOp(gamepad1, gamepad2);
        }
        stopDriving();

        //Stop the motor
        motor.setPower(0.0);

        //Use the encoder in the future
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveTimedWait(Gamepad gamepad1, Gamepad gamepad2, ElapsedTime timer, int waitTime)
    {
        timer.reset();
        while (timer.time() < waitTime)
        {
            chassisTeleOp(gamepad1, gamepad2);
        }
        stopDriving();
    }

    //Empty main
    @Override
    public void runOpMode() throws InterruptedException
    {
        float power = (float) 1.0;
        rightTurnAutonomous(power, 100);
        leftTurnAutonomous(power, 100);
        pidIMULeft(power, 90);
        pidIMURight(power, 90);
        coeffShiftTeleop(power);
        encoderCoeffLeftTurn(power, 90);
        encoderCoeffRightTurn(power, 90);
    }
} //Close class and end program

