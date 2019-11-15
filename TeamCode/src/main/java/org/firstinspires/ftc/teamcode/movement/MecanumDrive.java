package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Class representing four DcMotors that comprise a Mecanum drive; in other words, the drive we're using on our robot.
 */
public class MecanumDrive {
    Motor[] motors;

    /**
     * Class representing a motor used in a mecanum drive.
     */
    public static class Motor {
        String name = null;
        DcMotor motor;
        Vector2D vector;
        byte location = 0;

        /**
         * Class representing a 2x1 column vector, used for calculating what speeds to move the motors in.
         */
        public static class Vector2D {
            /**
             * The first element of the vector. (horizontal movement)
             */
            public final double x;

            /**
             * The second element of the vector. (vertical movement)
             */
            public final double y;

            /**
             * Constructs a vector based on a given x and y.
             * @param x the first element of the vector
             * @param y the second element of the vector
             */
            public Vector2D(double x, double y) {
                this.x = x;
                this.y = y;
            }

            /**
             * Computes the dot product of two Vector2Ds.
             * @param a the first operand
             * @param b the second operand
             * @return the dot product of a and b
             */
            public static double dotProduct(Vector2D a, Vector2D b) {
                return a.x * b.x + a.y * b.y;
            }

            /**
             * Returns a version of the vector rotated by a given angle in radians.
             * @param angle angle at which to rotate the vector, in radians
             * @return the rotated vector
             */
            public Vector2D rotatedBy(double angle) {
                double sinOf = Math.sin(angle);
                double cosOf = Math.cos(angle);
                return new MecanumDrive.Motor.Vector2D(x * cosOf - y * sinOf, x * sinOf + y * cosOf);
            }
        }

        /**
         * The location at which this motor is mounted on the robot, used to calculate which powers should be altered during a turn.
         */
        public enum Location {
            FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
        }

        /**
         * Constructs a Motor given a DcMotor, the vector in which it moves, and the location of the motor on the robot.
         * @param motor the DcMotor this Motor represents
         * @param vector the direction this Motor moves in
         * @param location the location at which this Motor is mounted on the physical robot
         */
        public Motor(DcMotor motor, Vector2D vector, Location location) {
            this.motor = motor;
            this.vector = vector;

            if (location == Location.FRONT_LEFT) {
                this.location = 2;
            } else if (location == Location.FRONT_RIGHT) {
                this.location = 2 | 1;
            } else if (location == Location.BACK_LEFT) {
                this.location = 0;
            } else if (location == Location.BACK_RIGHT) {
                this.location = 1;
            }
        }
    }

    /**
     * Constructs a MecanumDrive using the specified set of motors
     * @param motors the set of motors with which to construct this MecanumDrive
     */
    public MecanumDrive(Motor[] motors) {
        this.motors = motors;
    }

    /**
     * An enum representing what to do when a power exceeds a motor's limits.
     */
    public enum PowerBehavior {
        /**
         * Clamp the power to the maximum allowed limits.
         */
        CLAMP,

        /**
         * Divide the power by 2.
         */
        DIVIDE
    }

    /**
     * What to do when a power exceeds a motor's limits.
     */
    public PowerBehavior powerBehavior = PowerBehavior.CLAMP;

    private double transformPower(double power) {
        return powerBehavior == PowerBehavior.CLAMP ? Range.clip(power, -1.0, 1.0) : power / 2;
    }

    public DcMotor[] getMotors() {
        DcMotor[] dcMotors = new DcMotor[motors.length];
        for (int i = 0; i < motors.length; i++) {
            dcMotors[i] = motors[i].motor;
        }
        return dcMotors;
    }

    /**
     * Sets each of the drive's motors to a given mode.
     * @param mode the mode to set each motor to
     */
    public void setMotorMode(DcMotor.RunMode mode) {
        for (DcMotor motor: getMotors()) {
            motor.setMode(mode);
        }
    }

    /**
     * An enum representing how turning should be calculated.
     */
    public enum TurnBehavior {
        /**
         * Multiply the power by the turn value.
         */
        MULTIPLY,

        /**
         * Add/subtract the turn value from the power.
         */
        ADDSUBTRACT
    }

    /**
     * Moves in a given direction at a given power while turning towards a given direction
     * @param power the power to move in
     * @param vector the direction to move in
     * @param turn the direction at which to turn
     * @param turnBehavior how turning should be calculated
     */
    public void move(double power, Motor.Vector2D vector, double turn, TurnBehavior turnBehavior) {
        for (Motor motor: motors) {
            double transformed = transformPower(Motor.Vector2D.dotProduct(vector, motor.vector)) * power;
            if ((motor.location & 1) > 0) {
                double motorPower = turnBehavior == TurnBehavior.ADDSUBTRACT ? transformed - turn : transformed * Range.clip(1.0 - 2.0 * turn, -1.0, 1.0);
                motor.motor.setPower(Range.clip(motorPower, -1.0, 1.0));
            } else {
                double motorPower = turnBehavior == TurnBehavior.ADDSUBTRACT ? transformed + turn : transformed * Range.clip(1.0 + 2.0 * turn, -1.0, 1.0);
                motor.motor.setPower(Range.clip(motorPower, -1.0, 1.0));
            }
        }
    }

    /**
     * Moves in a given direction at a given power while turning towards a given direction. Uses MULTIPLY for the turn behavior.
     * @param power the power to move in
     * @param vector the direction to move in
     * @param turn the direction at which to turn
     */
    public void move(double power, Motor.Vector2D vector, double turn) {
        move(power, vector, turn, TurnBehavior.MULTIPLY);
    }

    public void forwardWithPower(double power) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(power, new Motor.Vector2D(0, 1), 0);
    }

    public void forwardWithSpeed(double power) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move(power, new Motor.Vector2D(0, 1), 0);
    }

    /**
     * Strafes left at a given power (i.e. without encoder).
     * @param power the power at which to drive the motors
     */
    public void strafeLeftWithPower(double power) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(power, new Motor.Vector2D(-1, 0), 0);
    }

    /**
     * Strafes left at a given speed (i.e. with encoder).
     * @param speed the speed at which to drive the motors
     */
    public void strafeLeftWithSpeed(double speed) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move(speed, new Motor.Vector2D(-1, 0), 0);
    }

    /**
     * Strafes right at a given power (i.e. without encoder).
     * @param power the power at which to drive the motors
     */
    public void strafeRightWithPower(double power) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(power, new Motor.Vector2D(1, 0), 0);
    }

    /**
     * Strafes right at a given speed (i.e. with encoder).
     * @param speed the speed at which to drive the motors
     */
    public void strafeRightWithSpeed(double speed) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(speed, new Motor.Vector2D(1, 0), 0);
    }

    public void steerWithPower(double power, double turn) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(power, new Motor.Vector2D(0, 1), turn);
    }

    public void steerWithSpeed(double speed, double turn) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move(speed, new Motor.Vector2D(0, 1), turn);
    }

    public void stop() {
        for (DcMotor motor: getMotors()) {
            motor.setPower(0);
        }
    }

    /**
     * Stops and resets each of the motors and their encoders.
     */
    public void reset() {
        for (DcMotor motor: getMotors()) {
            motor.setPower(0);
        }

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    static String[] standardMotorNames = {"mecanum_fl", "mecanum_fr", "mecanum_bl", "mecanum_br"};
    static Motor.Vector2D[] standardMotorVectors = {new Motor.Vector2D(1, 1), new Motor.Vector2D(-1, 1), new Motor.Vector2D(-1, 1), new Motor.Vector2D(1, 1)};
    static Motor.Location[] standardMotorLocations = {Motor.Location.FRONT_LEFT, Motor.Location.FRONT_RIGHT, Motor.Location.BACK_LEFT, Motor.Location.BACK_RIGHT};
    static DcMotor.Direction[] standardMotorDirections = {DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD};

    /**
     * Given only a hardware map, constructs a ready-to-use mecanum drive with preset motors.
     * @param hardwareMap hardware map containing the motors installed on the robot
     * @return a "standard" ready-to-use mecanum drive
     */
    public static MecanumDrive standard(HardwareMap hardwareMap) {
        Motor[] motors = new Motor[standardMotorNames.length];
        for (int i = 0; i < motors.length; i++) {
            DcMotor motor = hardwareMap.get(DcMotor.class, standardMotorNames[i]);
            motor.setDirection(standardMotorDirections[i]);
            motors[i] = new Motor(motor, standardMotorVectors[i], standardMotorLocations[i]);
        }
        return new MecanumDrive(motors);
    }
}