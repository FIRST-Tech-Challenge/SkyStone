package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

//TO BE CLEANED UP
//ONLY REMAINING IS setRevese and Error methods not moved to Chassis.


/**
 * Base definition of Subsystems. Subsystems are identifiable sections of robot
 * that contain motors or servos
 *
 */
public class Subsystem {
    /**
     * Definition of arrays of types of motors used
     */
    DcMotor[] motors;
    Servo[] servos;

    /**
     * Constructors of the motors
     * @param motors
     */
    public void initMotors(DcMotor[] motors) {
        this.motors = motors;
    }

    public void initServos(Servo[] servos) {
        this.servos = servos;
    }

    //TO DELETE? NOT USED ANYWHERE
    /**
     * Methods for ServoMotors
     */
    /**
     * Sets servomotor to correct position between 0 and 1
     * When physical set up of motor is done, the 0 and/or 1 position has to be caliberated
     * by totating motor to desired value. Use tools like RevHubInterface for set up of
     * the servomotors to known 0 and/or 1 position.
     * @param position is the desired value to which the servo rotates on calling the
     *                 setPosition method
     */
    public void setServoPosition(double position) {
        for (Servo servo : servos) {
            servo.setPosition(position);
        }
    }

    /**
     * Methods for DCMotors
     */
    /**
     * Set the DC motor power to passed value.
     * @param power
     */
    public void setMotorPowers(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    //TO DELETE? IS HASHMAP USED?
    public HashMap<String, Integer> getMotorPositions() {
        HashMap<String, Integer> positions = new HashMap<>();
        for (DcMotor motor : motors) {
            positions.put(motor.getDeviceName(), motor.getCurrentPosition());
        }
        return positions;
    }

    /**
     * Function to set the behaviour of the motor on passing Zero power to the motor
     * @param zeroPowerBehavior could be BRAKE or FLOAT. When not defined, it is set
     *                          to UNKNOWN state, which is not desired.
     */
    public void setZeroBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Function to the direction of all DC motors in the subsystem
     * @param reverseMotors
     */
    public void reverseMotors(DcMotor[] reverseMotors) {
        for (DcMotor reverseMotor : reverseMotors) {
            reverseMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    /**
     * Reset function for motor encoders to be set to reset state of encoder.
     * Usage of this is typically followed by using setZeroBehaviour and then setting
     * the mode for th motor
     */
    public void reset() {

        for (DcMotor motor : motors) {
            DcMotor.RunMode runMode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(runMode);
        }
    }

    /**
     * Set the mode of the DC motor to RUN_WITHOUT_ENCODER (run at achievable velocity
     * RUN_USING_ENCODER (run at a targeted velocity or RUN_TO_POSITION (PID based rotation to
     * achieve the desited encoder count
     * @param runMode
     */
    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    //TO DELETE? Not used anywhere.
    /**
     * Get the absolute error in position between current and targeted position
     * To be used for usecases like implementing own PID or testing if a target position
     * is achieved.
     * @return
     */
    public int[] getMotorError() {
        int[] errors = new int[motors.length];
        for (int i = 0; i < errors.length; i++) {
            DcMotor motor = motors[i];
            errors[i] = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        }
        return errors;
    }

    /**
     * Find the average error across all motors (used in chassis movement)
     * Poll the error for each motor, sum all errors and divide by number of motors.
     * @return
     */
    public double getAverageMotorError() {
        int[] motorErrors = getMotorError();
        double sum = 0;
        for (int motorError : motorErrors) {
            sum += motorError;
        }
        return sum / motorErrors.length;
    }

    /**
     * Check if the current state of error is less than the desired target error value.
     * @param targetError
     * @return
     */
    public boolean motorErrorCheck(double targetError) {
        return targetError > getAverageMotorError();
    }


}
