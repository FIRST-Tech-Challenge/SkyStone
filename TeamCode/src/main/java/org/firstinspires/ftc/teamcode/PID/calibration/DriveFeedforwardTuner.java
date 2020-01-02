package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.tuning.AccelRegression;
import com.acmerobotics.roadrunner.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.util.LoggingUtil;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.getMaxRpm;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.rpmToVelocity;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Autonomous(name = "DriveFeedForwardTuner", group = "drive")
@Disabled
public class DriveFeedforwardTuner extends LinearOpMode {
    public static final double MAX_POWER = 0.7;
    //public static final double DISTANCE = 100;
    public static final double DISTANCE = DriveConstantsPID.TEST_DISTANCE;
    private String TAG = "DriveFeedforwardTuner";
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }
        RobotLog.dd(TAG, "DISTANCE: "+Double.toString(DISTANCE));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Press play to begin the feedforward tuning routine");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Would you like to fit kStatic?");
        telemetry.addLine("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitIntercept = false;
        while (!isStopRequested()) {
            if (gamepad1.a) {
                fitIntercept = true;
                while (!isStopRequested() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        telemetry.clearAll();
        telemetry.addLine(Misc.formatInvariant(
                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
        telemetry.addLine("Press (A) to begin");
        telemetry.update();

        while (!isStopRequested() && !gamepad1.a) {
            idle();
        }
        while (!isStopRequested() && gamepad1.a) {
            idle();
        }

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        double maxVel = rpmToVelocity(getMaxRpm());
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);
        RobotLog.dd(TAG, "maxVel: " + Double.toString(maxVel));
        RobotLog.dd(TAG, "finalVel: " + Double.toString(finalVel));
        RobotLog.dd(TAG, "accel: " + Double.toString(accel));
        RobotLog.dd(TAG, "rampTime: " + Double.toString(rampTime));

        double startTime = clock.seconds();
        RampRegression rampRegression = new RampRegression();

        drive.setPoseEstimate(new Pose2d());
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            Pose2d t = drive.getPoseEstimate();
            RobotLog.dd(TAG, "rampRegression\nelapsedTime: "+Double.toString(elapsedTime));
            RobotLog.dd(TAG, "X: " + Double.toString(t.getX()));
            RobotLog.dd(TAG, "Y: " + Double.toString(t.getY()));
            RobotLog.dd(TAG, "heading: " + Double.toString(t.getHeading()));
            RobotLog.dd(TAG, "accel: "+Double.toString(accel));
            RobotLog.dd(TAG, "vel(accel*elapsedTime): "+Double.toString(vel));
            RobotLog.dd(TAG, "Power(vel/maxVel): " + Double.toString(power));

            rampRegression.add(elapsedTime, t.getX(), power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

        RampRegression.RampResult rampResult = rampRegression.fit(fitIntercept);

        rampRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        telemetry.clearAll();
        telemetry.addLine("Quasi-static ramp up test complete");
        if (fitIntercept) {
            RobotLog.dd(TAG, "kV: " + Double.toString(rampResult.kV) +
                    " kStatic: " + Double.toString(rampResult.kStatic)+ " rSquare: "+Double.toString(rampResult.rSquare));

            telemetry.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
        } else {
            RobotLog.dd(TAG, " kS: " + Double.toString(rampResult.kStatic)+
                    " rSquare: "+Double.toString(rampResult.rSquare));

            telemetry.addLine(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
                    rampResult.kStatic, rampResult.rSquare));
        }
        telemetry.addLine("Would you like to fit kA?");
        telemetry.addLine("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitAccelFF = false;
        while (!isStopRequested()) {
            if (gamepad1.a) {
                fitAccelFF = true;
                RobotLog.dd(TAG,"fitAccelFF is true");
                while (!isStopRequested() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (!isStopRequested() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        if (fitAccelFF) {
            telemetry.clearAll();
            telemetry.addLine("Place the robot back in its starting position");
            telemetry.addLine("Press (A) to continue");
            telemetry.update();

            while (!isStopRequested() && !gamepad1.a) {
                idle();
            }
            while (!isStopRequested() && gamepad1.a) {
                idle();
            }

            telemetry.clearAll();
            telemetry.addLine("Running...");
            telemetry.update();

            double maxPowerTime = DISTANCE / maxVel;

            startTime = clock.seconds();
            AccelRegression accelRegression = new AccelRegression();

            drive.setPoseEstimate(new Pose2d());
            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));
            while (!isStopRequested()) {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }
                Pose2d t = drive.getPoseEstimate();
                accelRegression.add(elapsedTime, t.getX(), MAX_POWER);


                RobotLog.dd(TAG, "accelRegression\nelapsedTime: "+Double.toString(elapsedTime));
                RobotLog.dd(TAG, "X: " + Double.toString(t.getX()));
                RobotLog.dd(TAG, "Y: " + Double.toString(t.getY()));
                RobotLog.dd(TAG, "heading: " + Double.toString(t.getHeading()));
                RobotLog.dd(TAG, "MAX_POWER: "+Double.toString(MAX_POWER));

                drive.updatePoseEstimate();
            }
            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

            AccelRegression.AccelResult accelResult = accelRegression.fit(
                    rampResult.kV, rampResult.kStatic);

            RobotLog.dd(TAG, " kV: " + Double.toString(rampResult.kV)+ " kStatic: "+Double.toString(rampResult.kStatic));

            accelRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                    "DriveAccelRegression-%d.csv", System.currentTimeMillis())));

            telemetry.clearAll();
            telemetry.addLine("Constant power test complete");

            RobotLog.dd(TAG, " kA: " + Double.toString(accelResult.kA)+ " rSquare: "+Double.toString(accelResult.rSquare));

            telemetry.addLine(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare));
            telemetry.update();
        }

        while (!isStopRequested()) {
            idle();
        }
    }
}