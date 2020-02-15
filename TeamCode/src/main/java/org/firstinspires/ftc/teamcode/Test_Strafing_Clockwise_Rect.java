package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


@Autonomous(name = "Test_Strafing_Clockwise_Rect", group = "TestProgram")
public class Test_Strafing_Clockwise_Rect extends HardwareTestAuto
{

    int currentState = 0;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean firstTime = true;

    int StartFrontRightPosition;
    int StartFrontLeftPosition;

    // int startAngle;
    // int remaingingAngle;

    double speedBoost = 0; // random speed value, on top of a minimum base motor power

    // public static final double TURNING_POWER = 0.3;  // for gyro-based turn

    public static final double DRIVING_BASE_POWER = 0.1;     // keep at MIN, will add extra power
    public static final double STRAFING_BASE_POWER = 0.1;    // keep at MIN, will add extra power


    public enum Sides  // for sideways movement 
    {
        LEFT, RIGHT
    }

    public enum Direction // for forward-backward movement
    {
        FORWARD, BACKWARD
    }



    @Override
    public void init() 
    {

        super.init();
        
        /*
        // Calibrate the gyro.
        gyro.calibrate();
        */

        // get a random power to test robot navigation behavior in different speed  
        speedBoost = Math.random() * 0.5;   // to get max 0.5
        
        if (speedBoost > 0.5)  // set max limit, extra check
            speedBoost = 0.5;

        telemetry.addData("Motor Speed: ", (STRAFING_BASE_POWER + speedBoost));

    }


    public void init_loop() 
    {

    }

    public void start() 
    {
        currentState = 0;
        runtime.reset();
    }

    public void loop() 
    {

        telemetry.addData("CASE: ", currentState);
        telemetry.addData("TIME: ", runtime);
        telemetry.update();
        
        // this code uses state-machine logic. It basically executes the cases in incremental order
        // 
        // driveWithEncoderMotor4(): moves robot forward/backward
        //    runs all 4 motors with encoder count. First 2 parameters are encoder count to FRONT 2 motors, ideally should be same value
        //    speed value is passed as speedboast
        //    last parameter is timeout, should be high value to avoid timeout in this case
        //
        // strafeWithEncoders(): moves robot sideways
        //    first parameter specifies direction (Left/Right)
        //    2nd parameter is encoder count
        
        switch (currentState) {

            case 0: //robot going forward
                if (driveWithEncoderMotor4(1000, 1000, Direction.FORWARD, speedBoost, 10000)) {
                    currentState++;
                    firstTime = true;
                }
                break;

            case 1: //robot going right
                if (strafeWithEncoders(Sides.LEFT, 1000, speedBoost, 10000)){
                    currentState++;
                    firstTime = true;
                }
                break;

            case 2: //robot coming backward
                if (driveWithEncoderMotor4(1000,1000, Direction.BACKWARD, speedBoost,10000)) {
                    currentState++;
                    firstTime = true;
                }
                break;

            case 3: //robot going left
                if (strafeWithEncoders(Sides.RIGHT, 1000, speedBoost, 10000)){
                    currentState++;
                    firstTime = true;
                }
                break;

            default:
                currentState++;
                firstTime = true;
                break;
        }

        telemetry.addData("**** current currentState : ****", currentState);

    }

    @Override
    public void stop() 
    {

    }


    // driveWithEncoderMotor4(): moves robot forward/backward
    //    runs all 4 motors with encoder count. First 2 parameters are encoder count to FRONT 2 motors, ideally should be same value
    //    speed value is passed as speedboast
    //    last parameter is timeout, should be high value to avoid timeout in this case
    boolean driveWithEncoderMotor4(int countFR, int countFL, Direction direction, double powerAdd, int timeout)

    {
        double motorDrivePower = DRIVING_BASE_POWER + powerAdd;

    	// the flag "firstTime" is used to initiate the motor move by setting the power value (when it's true)
    	// the motor will be stopped when the encoder count is reached (which is checked below)  
        if (firstTime && runtime.milliseconds() > 100) 
        {
            StartFrontRightPosition = frontRight.getCurrentPosition();
            StartFrontLeftPosition = frontLeft.getCurrentPosition();

            if (direction == Direction.FORWARD) 
            {
                frontRight.setPower(motorDrivePower);
                frontLeft.setPower(motorDrivePower);
                backLeft.setPower(motorDrivePower);
                backRight.setPower(motorDrivePower);
            } else 
            {
                frontRight.setPower(-motorDrivePower);
                frontLeft.setPower(-motorDrivePower);
                backLeft.setPower(-motorDrivePower);
                backRight.setPower(-motorDrivePower);
            }
           
            // make it false to skip calling SetPower() subsequently
            firstTime = false;
        }

        // stop the motor when the encoder count reached the specified value
        if (firstTime == false && (encoderReached(countFR, frontRight, StartFrontRightPosition) ||
                encoderReached(countFL, frontLeft, StartFrontLeftPosition) ||
                (runtime.milliseconds() > timeout))) 
        {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            
            StartFrontRightPosition = 0;
            StartFrontLeftPosition = 0;
            runtime.reset();
            firstTime = true;
            
            return true;
        } 
        else
        {
            return false;
        }
    }

    //
    // check if encoder count reached
    // return true if it has reached the specified encoder count
    //
    public boolean encoderReached(int count, DcMotor dcMotor, int startPosition) 
    {
        // Assume failure.
        boolean l_return = false;

        if (dcMotor != null) 
        {
            // Has the encoder reached the specified values?
            if (Math.abs(dcMotor.getCurrentPosition() - startPosition) >= count) 
            {
                // Set the status to a positive indication.
                l_return = true;

            }
        }
        
        telemetry.addData("BackRight" + "encoderReached", Math.abs(backRight.getCurrentPosition() - startPosition));
        telemetry.addData("BackLeft" + "encoderReached", Math.abs(backLeft.getCurrentPosition() - startPosition));
        telemetry.addData(dcMotor + "encoderReached", Math.abs(dcMotor.getCurrentPosition() - startPosition));

        // Return the status.
        return l_return;
    }

    
    // strafeWithEncoders(): moves robot sideways
    //    first parameter specifies Straffing direction (Left/Right)
    //    2nd parameter is encoder count
    boolean strafeWithEncoders(Sides turnDirection, int count, double powerAdd, int timeout)
    {
        double motorStrafingPower = STRAFING_BASE_POWER + powerAdd;

        if (firstTime && runtime.milliseconds() > 50) 
        {
            StartFrontRightPosition = frontRight.getCurrentPosition();
            StartFrontLeftPosition = frontLeft.getCurrentPosition();

            if (turnDirection == Sides.LEFT) 
            {
                frontRight.setPower(-motorStrafingPower);
                frontLeft.setPower(motorStrafingPower);
                backLeft.setPower(-motorStrafingPower);
                backRight.setPower(motorStrafingPower);
            } 
            else 
            { //Right side
                frontRight.setPower(motorStrafingPower);
                frontLeft.setPower(-motorStrafingPower);
                backLeft.setPower(motorStrafingPower);
                backRight.setPower(-motorStrafingPower);
            }

            firstTime = false;
        }

        // check if any of the two encoder count (front left or front right) has reached specified limit
        // also stop motors when timeout occurs
        if (firstTime == false && (encoderReached(count, frontRight, StartFrontRightPosition) || encoderReached(count, frontLeft, StartFrontLeftPosition) || (runtime.milliseconds() > timeout))) 
        {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            firstTime = true;
            StartFrontRightPosition = 0;
            StartFrontLeftPosition = 0;
            runtime.reset();

            return true;
        } 
        else
        {
            return false;
        }

    }
    
    
    /*
    boolean gyroPointTurn(Sides turnDirection, int angle) {
        int progress;
        double power;

        if (firstTime) {
            gyro.resetZAxisIntegrator();
            firstTime = false;
        }

        progress = Math.abs(gyro.getIntegratedZValue() - startAngle);
        remaingingAngle = angle - progress;
        power = TURNING_POWER;

        if (turnDirection == Sides.LEFT) {
            frontRight.setPower(-power);
            frontLeft.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }
        if (turnDirection == Sides.RIGHT) {
            frontRight.setPower(power);
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }
        // Target is reached if remainingAngle is within threshold.. (2 degrees)
        if (remaingingAngle <= 2) {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            runtime.reset();
            firstTime = true;
            return true;
        } else
            return false;
    }
     */

}

