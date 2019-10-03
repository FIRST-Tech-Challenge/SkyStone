package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveControl {
    private LinearOpMode opmode = null;
    private DcMotor FrontRightM = null;
    private DcMotor FrontLeftM = null;
    private DcMotor BackRightM = null;
    private DcMotor BackLeftM = null;
    private double MAX_SPEED = 0.6;
    private ElapsedTime runtime = new ElapsedTime();

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;
        opMode.telemetry.addLine("Drive Control initialize");
        opMode.telemetry.update();

        opmode = opMode;
        hwMap = opMode.hardwareMap;
        FrontLeftM = hwMap.dcMotor.get("leftFrontMotor");
        FrontRightM = hwMap.dcMotor.get("rightFrontMotor");
        BackLeftM = hwMap.dcMotor.get("leftRearMotor");
        BackRightM = hwMap.dcMotor.get("rightRearMotor");

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
        moveMotorsTime(-speed, speed,
                speed, -speed, timeSEC);
    }

    public void strafeRightTime(double speed, double timeSEC) {
        moveMotorsTime(speed, -speed,
                -speed, speed, timeSEC);
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
        distance = (int) (distCM * 20.0);

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

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (!opmode.isStopRequested() &&
                (FrontLeftM.isBusy() || FrontRightM.isBusy() ||
                        BackLeftM.isBusy() || BackRightM.isBusy())) {

            // Display it for the driver.
            opmode.telemetry.addData("Start -> Target", "%7d :%7d", startFL, targetFL);
            opmode.telemetry.addData("Running", "%7d :%7d",
                    FrontLeftM.getCurrentPosition(),
                    FrontRightM.getCurrentPosition());
            opmode.telemetry.addData("Running", "%7d :%7d",
                    BackLeftM.getCurrentPosition(),
                    BackRightM.getCurrentPosition());
            opmode.telemetry.update();
            opmode.sleep(40);
        }

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
        moveMotorsDistance(-speed, speed,
                speed, -speed, distCM);
    }

    public void strafeRightDistance(double speed, double distCM) {
        moveMotorsDistance(speed, -speed,
                -speed, speed, distCM);
    }

    public void turnLeftDistance(double speed, double distCM) {
        moveMotorsDistance(-speed, speed,
                -speed, speed, distCM);
    }

    public void turnRightDistance(double speed, double distCM) {
        moveMotorsDistance(speed, -speed,
                speed, -speed, distCM);
    }
}
