package com.acmerobotics.roadrunner.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.hfrobots.tnt.corelib.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.stat.regression.SimpleRegression;

import java.util.ArrayList;
import java.util.List;

/**
 * Op mode for computing kV, kStatic, and kA from various drive routines.
 */
public abstract class FeedforwardTuningOpMode extends LinearOpMode {
    private static final double MAX_POWER = 0.7;
    private static final double EPSILON = 1e-2;

    private double distance;
    private double wheelMotorRpm;
    private double wheelDiameter;
    private double wheelGearRatio;

    /**
     * @param distance allowable forward travel distance
     * @param wheelMotorRpm wheel motor rpm
     * @param wheelDiameter wheel diameter
     * @param wheelGearRatio wheel gear ratio (output / input)
     */
    public FeedforwardTuningOpMode(double distance, double wheelMotorRpm, double wheelDiameter, double wheelGearRatio) {
        this.distance = distance;
        this.wheelMotorRpm = wheelMotorRpm;
        this.wheelDiameter = wheelDiameter;
        this.wheelGearRatio = wheelGearRatio;
    }

    public FeedforwardTuningOpMode(double distance, double wheelMotorRpm, double wheelDiameter) {
        this(distance, wheelMotorRpm, wheelDiameter, 1.0);
    }

    private static List<Double> numericalDerivative(List<Double> x, List<Double> y) {
        List<Double> deriv = new ArrayList<>();
        for (int i = 2; i < x.size(); i++) {
            deriv.add((y.get(i) - y.get(i-2)) / (x.get(i) - x.get(i-2)));
        }
        deriv.add(0, deriv.get(0));
        deriv.add(deriv.get(deriv.size() - 1));
        return deriv;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = initDrive();

        telemetry.log().add("Press play to begin the feedforward tuning routine");
        telemetry.update();

        waitForStart();

        telemetry.log().clear();
        telemetry.log().add("Would you like to fit kStatic?");
        telemetry.log().add("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitIntercept = false;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fitIntercept = true;
                while (opModeIsActive() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (opModeIsActive() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add(String.format("Place your robot on the field with at least %.2f in of room in front", distance));
        telemetry.log().add("Press (A) to begin");
        telemetry.update();

        while (opModeIsActive() && !gamepad1.a) {
            idle();
        }
        while (opModeIsActive() && gamepad1.a) {
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        double maxVel = wheelMotorRpm * wheelGearRatio * Math.PI * wheelDiameter / 60.0;
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * distance);
        double rampTime = Math.sqrt(2.0 * distance / accel);

        double startTime = System.nanoTime() / 1e9;
        List<Double> timeSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();

        drive.setPoseEstimate(new Pose2d());
        while (opModeIsActive()) {
            double elapsedTime = System.nanoTime() / 1e9 - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            powerSamples.add(power);
            positionSamples.add(drive.getPoseEstimate().getX());

            drive.setVelocity(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setVelocity(new Pose2d(0.0, 0.0, 0.0));

        List<Double> velocitySamples = numericalDerivative(timeSamples, positionSamples);
        SimpleRegression rampRegression = new SimpleRegression(fitIntercept);
        for (int i = 0; i < velocitySamples.size(); i++) {
            rampRegression.addData(velocitySamples.get(i), powerSamples.get(i));
        }
        double kV = rampRegression.getSlope();
        double kStatic = rampRegression.getIntercept();

        double maxSampledVelocity = Double.MIN_VALUE;

        for (Double val : velocitySamples) {
            if (val > maxSampledVelocity) {
                maxSampledVelocity = val;
            }
        }


        // here we will log the velocity samples
        logTimeseriesData(timeSamples, velocitySamples, "velocity");

        telemetry.log().clear();
        telemetry.log().add("Quasi-static ramp up test complete");
        if (fitIntercept) {
            telemetry.log().add(String.format("kV = %.5f, kStatic = %.5f (R^2 = %.2f)", kV, kStatic, rampRegression.getRSquare()));
        } else {
            telemetry.log().add(String.format("kV = %.5f (R^2 = %.2f) max = %.5F", kV, rampRegression.getRSquare(), maxSampledVelocity));
        }
        telemetry.log().add("Would you like to fit kA?");
        telemetry.log().add("Press (A) for yes, (B) for no");
        telemetry.update();

        boolean fitAccelFF = false;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fitAccelFF = true;
                while (opModeIsActive() && gamepad1.a) {
                    idle();
                }
                break;
            } else if (gamepad1.b) {
                while (opModeIsActive() && gamepad1.b) {
                    idle();
                }
                break;
            }
            idle();
        }

        if (fitAccelFF) {
            telemetry.log().clear();
            telemetry.log().add("Place the robot back in its starting position");
            telemetry.log().add("Press (A) to continue");
            telemetry.update();

            while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
            while (opModeIsActive() && gamepad1.a) {
                idle();
            }

            telemetry.log().clear();
            telemetry.log().add("Running...");
            telemetry.update();

            double maxPowerTime = distance / maxVel;

            startTime = System.nanoTime() / 1e9;
            timeSamples.clear();
            positionSamples.clear();

            drive.setPoseEstimate(new Pose2d());
            drive.setVelocity(new Pose2d(MAX_POWER, 0.0, 0.0));
            while (opModeIsActive()) {
                double elapsedTime = System.nanoTime() / 1e9 - startTime;
                if (elapsedTime > maxPowerTime) {
                    break;
                }

                timeSamples.add(elapsedTime);
                positionSamples.add(drive.getPoseEstimate().getX());

                drive.updatePoseEstimate();
            }
            drive.setVelocity(new Pose2d(0.0, 0.0, 0.0));

            velocitySamples = numericalDerivative(timeSamples, positionSamples);
            List<Double> accelerationSamples = numericalDerivative(timeSamples, velocitySamples);

            SimpleRegression maxPowerRegresiion = new SimpleRegression(false);
            for (int i = 0; i < accelerationSamples.size(); i++) {
                double velocityPower = kV * velocitySamples.get(i);
                if (Math.abs(velocityPower) > EPSILON) {
                    velocityPower += Math.signum(velocityPower) * kStatic;
                } else {
                    velocityPower = 0;
                }
                double accelerationPower = MAX_POWER - velocityPower;
                maxPowerRegresiion.addData(accelerationSamples.get(i), accelerationPower);
            }
            double kA = maxPowerRegresiion.getSlope();

            double maxSampledAccel = Double.MIN_VALUE;

            for (Double val : accelerationSamples) {
                if (val > maxSampledAccel) {
                    maxSampledAccel = val;
                }
            }


            // Here, we want to log acceleration vs time
            logTimeseriesData(timeSamples, accelerationSamples, "acceleration");

            telemetry.log().clear();
            telemetry.log().add("Max power test complete");
            telemetry.log().add(String.format("kA = %.5f (R^2 = %.2f) max=%.5f", kA, maxPowerRegresiion.getRSquare(), maxSampledAccel));
            telemetry.update();
        }

        while (opModeIsActive()) {
            idle();
        }
    }

    protected abstract Drive initDrive();

    protected void logTimeseriesData(final List<Double> timeSamples,
                                     final List<Double> otherSamples,
                                     final String sampleName) {
        int numSamples = Math.max(otherSamples.size(), timeSamples.size());

        StringBuilder timeAndOtherSamples = new StringBuilder();

        for (int i = 0; i < numSamples; i++) {
            Double time = timeSamples.get(i);
            Double acceleration = otherSamples.get(i);

            timeAndOtherSamples.append(time).append(", ").append(acceleration).append("\n"); // what does \n do?
        }

        android.util.Log.d(Constants.LOG_TAG, sampleName + ": \n"  + timeAndOtherSamples.toString());
    }
}