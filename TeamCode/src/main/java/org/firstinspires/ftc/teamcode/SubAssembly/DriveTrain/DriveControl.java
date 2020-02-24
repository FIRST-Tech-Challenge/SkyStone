package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.ColorControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.IMUcontrol;

public class DriveControl {
    private LinearOpMode opmode = null;
    private DcMotorEx FrontRightM = null;
    private DcMotorEx FrontLeftM = null;
    private DcMotorEx BackRightM = null;
    private DcMotorEx BackLeftM = null;
    private final double MAX_SPEED = 0.8;
    private final double GEARING = (2.0 / 3.0);
    private final double ENCODER_LINES = 1120.0;
    private final double WHEEL_CIRCUMFERENCE_CM = (3.1415 * (4 * 2.54));
    private final double ROBOT_RADIUS_CM = 103.0;
    private final double CONVERT_CM_TO_ENCODER = GEARING * ENCODER_LINES / WHEEL_CIRCUMFERENCE_CM;
    private final double CONVERT_DEG_TO_CM = (360.0 / (2 * 3.1415 * ROBOT_RADIUS_CM));
    private final double RUN_TO_TOLERANCE_CM = 1.0;
    private final double STRAFE_SCALING = 4.0 / 3.0;
    private final double P_GAIN = 10.0;
    private final double I_GAIN = 10.0;
    private double P_GAIN_MOTOR_DEFAULT;
    private double I_GAIN_MOTOR_DEFAULT;
    private ElapsedTime runtime = new ElapsedTime();

    // public sensors
    public ColorControl Color = new ColorControl();
    public IMUcontrol IMU = new IMUcontrol();

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;
        opMode.telemetry.addLine("Drive Control initialize");
        opMode.telemetry.update();

        opmode = opMode;
        hwMap = opMode.hardwareMap;
        FrontLeftM = (DcMotorEx) hwMap.dcMotor.get("leftFrontMotor");
        FrontRightM = (DcMotorEx) hwMap.dcMotor.get("rightFrontMotor");
        BackLeftM = (DcMotorEx) hwMap.dcMotor.get("leftRearMotor");
        BackRightM = (DcMotorEx) hwMap.dcMotor.get("rightRearMotor");

        // Set the drive motor direction
        FrontLeftM.setDirection(DcMotor.Direction.FORWARD);
        FrontRightM.setDirection(DcMotor.Direction.REVERSE);
        BackLeftM.setDirection(DcMotor.Direction.FORWARD);
        BackRightM.setDirection(DcMotor.Direction.REVERSE);

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        //setting power now means setting the speed
        FrontLeftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // stop all motors
        FrontRightM.setPower(0);
        FrontLeftM.setPower(0);
        BackRightM.setPower(0);
        BackLeftM.setPower(0);

        // adjust PID gains
        PIDFCoefficients pid;
        pid = FrontLeftM.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        P_GAIN_MOTOR_DEFAULT = pid.p;
        I_GAIN_MOTOR_DEFAULT = pid.i;
        pid.p = P_GAIN;
        pid.i = I_GAIN;
        FrontLeftM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        FrontRightM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        BackLeftM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        BackRightM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);

        /* change tolerance for RUN_TO_POSITION */
        int tolerance = (int) (RUN_TO_TOLERANCE_CM * CONVERT_CM_TO_ENCODER);
        opmode.telemetry.addData("Motor tolerance ", tolerance);
        FrontLeftM.setTargetPositionTolerance(tolerance);
        FrontRightM.setTargetPositionTolerance(tolerance);
        BackLeftM.setTargetPositionTolerance(tolerance);
        BackRightM.setTargetPositionTolerance(tolerance);

        Color.init(opMode);
        IMU.init(opMode);
    }

    public void PIDTelemetry() {
        // get the PID coefficients for the RUN_USING_ENCODER modes.
        PIDFCoefficients pid;
        pid = FrontLeftM.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        opmode.telemetry.addData("FL PID", "%.04f, %.04f, %.0f", pid.p, pid.i, pid.d);
    }

    public void PIDIncrement(double p, double i, double d) {
        final double INC_P = 0.1;
        final double INC_I = 0.1;
        final double INC_D = 0.1;
        // get current PID coefficients
        PIDFCoefficients pid;
        pid = FrontLeftM.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        // change coefficients using methods included with DcMotorEx class.
        pid.p += p * INC_P;
        pid.i += i * INC_I;
        pid.d += d * INC_D;
        FrontLeftM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        FrontRightM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        BackLeftM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        BackRightM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
    }

    public void PIDReset(boolean default_initial) {
        PIDFCoefficients pid;
        pid = FrontLeftM.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (default_initial) {
            pid.p = P_GAIN_MOTOR_DEFAULT;
            pid.i = I_GAIN_MOTOR_DEFAULT;
        } else {
            pid.p = P_GAIN;
            pid.i = I_GAIN;
        }
        FrontLeftM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        FrontRightM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        BackLeftM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        BackRightM.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
    }

    // delays for a fixed number of seconds
    public void TimeDelay(double delayTimeSEC) {
        double startTime = 0;
        double elapsedTime = 0;
        startTime = runtime.seconds();
        do {
            elapsedTime = runtime.seconds() - startTime;
            opmode.sleep(40);
        } while ((elapsedTime < delayTimeSEC) && !opmode.isStopRequested());
    }

    // limits speed to +/- maximum speed
    public double limitSpeed(double speed) {
        return Math.max(-MAX_SPEED, Math.min(speed, MAX_SPEED));
    }

    // limits speed to 0, maximum speed
    public double limitSpeedPositive(double speed) {
        return Math.max(0.0, Math.min(speed, MAX_SPEED));
    }

    // stops all motors
    public void stop() {
        FrontLeftM.setPower(0);
        FrontRightM.setPower(0);
        BackLeftM.setPower(0);
        BackRightM.setPower(0);
    }

    // general motion method
    // starts all motors at specified speeds
    // returns immediately - does not wait and does not stop motors
    public void moveMotors(double speedFL, double speedFR,
                           double speedBL, double speedBR) {
        speedFL = limitSpeed(speedFL);
        speedFR = limitSpeed(speedFR);
        speedBL = limitSpeed(speedBL);
        speedBR = limitSpeed(speedBR);
        FrontLeftM.setPower(speedFL);
        FrontRightM.setPower(speedFR);
        BackLeftM.setPower(speedBL);
        BackRightM.setPower(speedBR);
    }

    public void moveForward(double speed) {
        moveMotors(speed, speed,
                speed, speed);
    }

    public void moveBackward(double speed) {
        moveMotors(-speed, -speed,
                -speed, -speed);
    }

    public void strafeLeft(double speed) {
        moveMotors(-speed, speed,
                speed, -speed);
    }

    public void strafeRight(double speed) {
        moveMotors(speed, -speed,
                -speed, speed);
    }

    public void turnLeft(double speed) {
        moveMotors(-speed, speed,
                -speed, speed);
    }

    public void turnRight(double speed) {
        moveMotors(speed, -speed,
                speed, -speed);
    }

    // timed motion methods
    // starts all motors at specified speeds
    // waits specified time then stops motors
    public void moveMotorsTime(double speedFL, double speedFR,
                               double speedBL, double speedBR, double timeSEC) {
        moveMotors(speedFL, speedFR,
                speedBL, speedBR);
        TimeDelay(timeSEC);
        stop();
    }

    public void moveForwardTime(double speed, double timeSEC) {
        moveMotorsTime(speed, speed,
                speed, speed, timeSEC);
    }

    public void moveBackwardTime(double speed, double timeSEC) {
        moveMotorsTime(-speed, -speed,
                -speed, -speed, timeSEC);
    }

    public void strafeLeftTime(double speed, double timeSEC) {
        double angle = IMU.getAngle();
        moveMotorsTime(-speed, speed,
                speed, -speed, timeSEC);
        turnToAngle(speed, angle);
    }

    public void strafeRightTime(double speed, double timeSEC) {
        double angle = IMU.getAngle();
        moveMotorsTime(speed, -speed,
                -speed, speed, timeSEC);
        turnToAngle(speed, angle);
    }

    public void turnLeftTime(double speed, double timeSEC) {
        moveMotorsTime(-speed, speed,
                -speed, speed, timeSEC);
    }

    public void turnRightTime(double speed, double timeSEC) {
        moveMotorsTime(speed, -speed,
                speed, -speed, timeSEC);
    }

    // distance motion methods
    // starts all motors at specified speeds
    // waits until motors move specified distance then stops motors
    public void moveMotorsDistance(double speedFL, double speedFR,
                                   double speedBL, double speedBR, double distCM) {
        int startFL, targetFL;
        int startFR, targetFR;
        int startBL, targetBL;
        int startBR, targetBR;
        int distance;

        // convert distance in cm to encoder value
        distance = (int) (distCM * CONVERT_CM_TO_ENCODER);

        // Determine new target position, and pass to motor controller
        startFL = FrontLeftM.getCurrentPosition();
        startFR = FrontRightM.getCurrentPosition();
        startBL = BackLeftM.getCurrentPosition();
        startBR = BackRightM.getCurrentPosition();
        if (speedFL > 0) targetFL = startFL + distance;
        else targetFL = startFL - distance;
        if (speedFR > 0) targetFR = startFR + distance;
        else targetFR = startFR - distance;
        if (speedBL > 0) targetBL = startBL + distance;
        else targetBL = startBL - distance;
        if (speedBR > 0) targetBR = startBR + distance;
        else targetBR = startBR - distance;
        FrontLeftM.setTargetPosition(targetFL);
        FrontRightM.setTargetPosition(targetFR);
        BackLeftM.setTargetPosition(targetBL);
        BackRightM.setTargetPosition(targetBR);

        // Turn On RUN_TO_POSITION
        FrontLeftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        speedFL = limitSpeed(speedFL);
        speedFR = limitSpeed(speedFR);
        speedBL = limitSpeed(speedBL);
        speedBR = limitSpeed(speedBR);
        FrontLeftM.setPower(speedFL);
        FrontRightM.setPower(speedFR);
        BackLeftM.setPower(speedBL);
        BackRightM.setPower(speedBR);

        // Display it for the driver.
        opmode.telemetry.addData("Start -> Target", "%7d :%7d", startFL, targetFL);
        opmode.telemetry.update();

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        boolean isBusy;
        do {
            // if 3 or more motors are busy, then we are busy
//          if ((FrontLeftM.isBusy() ? 1 : 0) + (FrontRightM.isBusy() ? 1 : 0) +
//              (BackLeftM.isBusy() ? 1 : 0) + (BackRightM.isBusy() ? 1 : 0) >= 3)
            if (FrontLeftM.isBusy() || FrontRightM.isBusy() || BackLeftM.isBusy() || BackRightM.isBusy())
                isBusy = true;
            else
                isBusy = false;
//            opmode.telemetry.addData("FL error ", FrontLeftM.getCurrentPosition() - FrontLeftM.getTargetPosition());
//            opmode.telemetry.addData("FR error ", FrontRightM.getCurrentPosition() - FrontRightM.getTargetPosition());
//            opmode.telemetry.addData("BL error ", BackLeftM.getCurrentPosition() - BackLeftM.getTargetPosition());
//            opmode.telemetry.addData("BR error ", BackRightM.getCurrentPosition() - BackRightM.getTargetPosition());
//            opmode.telemetry.update();
            opmode.sleep(40);
        } while (!opmode.isStopRequested() && isBusy);

        // Stop all motion
        FrontLeftM.setPower(0);
        FrontRightM.setPower(0);
        BackLeftM.setPower(0);
        BackRightM.setPower(0);

        // Turn off RUN_TO_POSITION
        FrontLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForwardDistance(double speed, double distCM) {
        moveMotorsDistance(speed, speed,
                speed, speed, distCM);
    }

    public void moveBackwardDistance(double speed, double distCM) {
        moveMotorsDistance(-speed, -speed,
                -speed, -speed, distCM);
    }

    public void strafeLeftDistance(double speed, double distCM) {
        double angle = IMU.getAngle();
        distCM *= STRAFE_SCALING;
        moveMotorsDistance(-speed, speed,
                speed, -speed, distCM);
        turnToAngle(speed, angle);
    }

    public void strafeRightDistance(double speed, double distCM) {
        double angle = IMU.getAngle();
        distCM *= STRAFE_SCALING;
        moveMotorsDistance(speed, -speed,
                -speed, speed, distCM);
        turnToAngle(speed, angle);
    }

    public void turnLeftAngle(double speed, double angleDEG) {
        double distCM = CONVERT_DEG_TO_CM * angleDEG;
        moveMotorsDistance(-speed, speed,
                -speed, speed, distCM);
    }

    public void turnRightAngle(double speed, double angleDEG) {
        double distCM = CONVERT_DEG_TO_CM * angleDEG;
        moveMotorsDistance(speed, -speed,
                speed, -speed, distCM);
    }

    public void turnToAngle(double speed, double toangleDEG) {
        double angleDEG;
        angleDEG = toangleDEG - IMU.getAngle();
        while (angleDEG > 180.0) angleDEG -= 360.0;
        while (angleDEG < -180.0) angleDEG += 360.0;
        if (angleDEG < 0.0)
            turnRightAngle(speed, -angleDEG);
        else
            turnLeftAngle(speed, angleDEG);
    }

    public void driveUntilColor(double speed) {
        // return immediately if color detected
        if (Color.isBlue() || Color.isRed())
            return;
        // start moving forward ...
        moveForward(speed);
        do {
            // debugging display
            Color.Telemetry();
            opmode.telemetry.update();

            opmode.sleep(40);
            // ... until color is detected of stop requested
        } while (!Color.isBlue() && !Color.isRed() && !opmode.isStopRequested());
        stop();
    }
}
