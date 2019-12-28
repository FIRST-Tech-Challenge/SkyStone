package org.firstinspires.ftc.teamcode.Experimental.Units;

import java.util.ArrayList;

public class AccelerationProperties {
    private double accelerationProportion;  //AccProportion: % of movement which the robot accelerates/decelerates (0 - 0.5)
    private double accelerationRate;    //AccRate: Accelerates by this much motor power per increment (0 - 1)
    private String debug;

    public AccelerationProperties(double proportion, double rate) {
        if (proportion > 0.5 || proportion < 0 || rate > 1 || rate < 0)
            throw new IllegalArgumentException("AccelerationProportion must be >= 0 and <= 0.5 and \n AccelerationRate must " +
                    "be >= 0 and <= 1");
        accelerationProportion = proportion;
        accelerationRate = rate;
    }

    public void accelerate(ConstraintsAndConstants constants, double initEncoderAvg,
                           double targetPower, double targetDistanceInches) {
        boolean atTargetDistance = false;
        double currentMotorPower = getAccelerationPower(targetPower);
        double accelerationFactor = getAccelerationPower(targetPower);
        double segmentCounter = 1;

        debug = System.currentTimeMillis() + ": \"Accelerate\" -- RateOfMotorPowerIncrease = " + accelerationFactor +
                " // EncoderCountsPerAcceleration = " +
                encoderCountsPerAcceleration(constants.wheelInchesToEncoderTicks(targetDistanceInches)) +
                " // NumOfAccSegments = " + getNumOfAccelerationSegments() + "\n";

        while (!atTargetDistance) {
            double currAvg = (constants.frontLeft.getCurrentPosition() + constants.frontRight.getCurrentPosition() + constants.backLeft.getCurrentPosition() +
                    constants.backRight.getCurrentPosition()) / 4;

            constants.frontLeft.setPower(currentMotorPower);
            constants.frontRight.setPower(currentMotorPower);
            constants.backLeft.setPower(currentMotorPower);
            constants.backRight.setPower(currentMotorPower);

            if (Math.abs(currAvg - initEncoderAvg) >=
                    encoderCountsPerAcceleration(constants.wheelInchesToEncoderTicks(targetDistanceInches)) * segmentCounter) {
                if (segmentCounter == getNumOfAccelerationSegments())
                    atTargetDistance = true;
                else {
                    segmentCounter += 1;
                    currentMotorPower += accelerationFactor;
                }
            }
        }
    }

    public void decelerate(ConstraintsAndConstants constants, double initEncoderAvg,
                           double currentPower, double targetDistanceInches) {
        boolean atTargetDistance = false;
        double currentMotorPower = getAccelerationPower(currentPower);
        double decelerationFactor = getAccelerationPower(currentPower);
        double segmentCounter = 1;
        double decelerateInitPoint = constants.wheelInchesToEncoderTicks((accelerationProportion +
                (1 - accelerationProportion * 2)) * targetDistanceInches);

        debug = System.currentTimeMillis() + ": \"Decelerate\" -- RateOfMotorPowerIncrease = " + decelerationFactor +
                " // EncoderCountsPerAcceleration = " +
                encoderCountsPerAcceleration(constants.wheelInchesToEncoderTicks(targetDistanceInches)) +
                " // NumOfDecSegments = " + getNumOfAccelerationSegments() + " // decelerateInitPoint = " +
                decelerateInitPoint + "\n";

        while (!atTargetDistance) {
            double currAvg = (constants.frontLeft.getCurrentPosition() + constants.frontRight.getCurrentPosition() + constants.backLeft.getCurrentPosition() +
                    constants.backRight.getCurrentPosition()) / 4;

            constants.frontLeft.setPower(currentMotorPower);
            constants.frontRight.setPower(currentMotorPower);
            constants.backLeft.setPower(currentMotorPower);
            constants.backRight.setPower(currentMotorPower);

            if (Math.abs(currAvg - initEncoderAvg) >= decelerateInitPoint +
                    encoderCountsPerAcceleration(constants.wheelInchesToEncoderTicks(targetDistanceInches)) * segmentCounter) {
                if (segmentCounter == getNumOfAccelerationSegments()) {
                    atTargetDistance = true;
                    constants.frontLeft.setPower(0);
                    constants.frontRight.setPower(0);
                    constants.backLeft.setPower(0);
                    constants.backRight.setPower(0);
                } else {
                    segmentCounter += 1;
                    currentMotorPower -= decelerationFactor;
                }
            }
        }
    }

    public void accelerate(ConstraintsAndConstants constants, double[] initEncoderCounts,
                           double[] targetPower, double[] targetDistanceInches) {
        if (initEncoderCounts.length != 4 || targetPower.length != 4 || targetDistanceInches.length != 4)
            throw new IllegalArgumentException("InitEncoderCounts[], TargetPower[], and TargetDistanceInches[] must" +
                    " each have a size = 4 in accelerate. \n (Each index holding data about frontL, frontR, backL, " +
                    "and backR motors in this particular order)");

        boolean atTargetDistance = false;
        double[] currentMotorPower = new double[targetPower.length];
        double[] segmentCounter = new double[]{1, 1, 1, 1};
        double[] accelerationFactor = new double[targetPower.length];

        for (int i = 0; i < currentMotorPower.length; i++) {
            currentMotorPower[i] = getAccelerationPower(targetPower[i]);
            accelerationFactor[i] = getAccelerationPower(targetPower[i]);
        }

        debug = System.currentTimeMillis() + ": \"Accelerate\" -- RateOfMotorPowerIncrease: " + arrayToString(accelerationFactor) +
                "// EncoderCountsPerAcceleration: " +
                arrayTargetDistanceToString(constants, targetDistanceInches) + "// NumOfAccSegments: " +
                getNumOfAccelerationSegments() + "\n";

        while (!atTargetDistance) {
            double[] currEncoderCounts = new double[]{constants.frontLeft.getCurrentPosition(), constants.frontRight.getCurrentPosition(),
                    constants.backLeft.getCurrentPosition(), constants.backRight.getCurrentPosition()};

            constants.frontLeft.setPower(currentMotorPower[0]);
            constants.frontRight.setPower(currentMotorPower[1]);
            constants.backLeft.setPower(currentMotorPower[2]);
            constants.backRight.setPower(currentMotorPower[3]);

            for (int i = 0; i < currEncoderCounts.length; i++) {
                if (Math.abs(currEncoderCounts[i] - initEncoderCounts[i]) >=
                        encoderCountsPerAcceleration(constants.wheelInchesToEncoderTicks(targetDistanceInches[i])) * segmentCounter[i]) {
                    if (segmentCounter[i] == getNumOfAccelerationSegments())
                        atTargetDistance = true;
                    else {
                        segmentCounter[i] += 1;
                        currentMotorPower[i] += accelerationFactor[i];
                    }
                }
            }
        }
    }

    public void decelerate(ConstraintsAndConstants constants, double[] initEncoderCounts,
                           double[] currentPower, double[] targetDistanceInches) {
        if (initEncoderCounts.length != 4 || currentPower.length != 4 || targetDistanceInches.length != 4)
            throw new IllegalArgumentException("InitEncoderCounts[], TargetPower[], and TargetDistanceInches[] must" +
                    " each have a size = 4 in decelerate. \n (Each index holding data about frontL, frontR, backL, " +
                    "and backR motors in this particular order)");

        boolean atTargetDistance = false;
        double[] currentMotorPower = new double[currentPower.length];
        double[] segmentCounter = new double[]{1, 1, 1, 1};
        double[] decelerateInitPoint = new double[targetDistanceInches.length];
        double[] decelerationFactor = new double[currentPower.length];

        for (int i = 0; i < targetDistanceInches.length; i++) {
            decelerateInitPoint[i] = constants.wheelInchesToEncoderTicks((accelerationProportion +
                    (1 - accelerationProportion * 2)) * targetDistanceInches[i]);
        }

        for (int i = 0; i < currentMotorPower.length; i++) {
            currentMotorPower[i] = getAccelerationPower(currentPower[i]);
            decelerationFactor[i] = getAccelerationPower(currentPower[i]);
        }

        debug = System.currentTimeMillis() + ": \"Decelerate\" -- RateOfMotorPowerIncrease: " + arrayToString(decelerationFactor) +
                " // EncoderCountsPerAcceleration: " +
                arrayTargetDistanceToString(constants, targetDistanceInches) + " // NumOfDecSegments: " +
                getNumOfAccelerationSegments() + "\n";

        while (!atTargetDistance) {
            double[] currEncoderCounts = new double[]{constants.frontLeft.getCurrentPosition(), constants.frontRight.getCurrentPosition(),
                    constants.backLeft.getCurrentPosition(), constants.backRight.getCurrentPosition()};

            constants.frontLeft.setPower(currentMotorPower[0]);
            constants.frontRight.setPower(currentMotorPower[1]);
            constants.backLeft.setPower(currentMotorPower[2]);
            constants.backRight.setPower(currentMotorPower[3]);

            for (int i = 0; i < currEncoderCounts.length; i++) {
                if (Math.abs(currEncoderCounts[i] - initEncoderCounts[i]) >= decelerateInitPoint[i] +
                        encoderCountsPerAcceleration(constants.wheelInchesToEncoderTicks(targetDistanceInches[i])) * segmentCounter[i]) {
                    if (segmentCounter[i] == getNumOfAccelerationSegments()) {
                        atTargetDistance = true;
                        constants.frontLeft.setPower(0);
                        constants.frontRight.setPower(0);
                        constants.backLeft.setPower(0);
                        constants.backRight.setPower(0);
                    } else {
                        segmentCounter[i] += 1;
                        currentMotorPower[i] -= decelerationFactor[i];
                    }
                }
            }
        }
    }

    public double encoderCountsPerAcceleration(double totalEncoderTicks) {
        return totalEncoderTicks * accelerationProportion * accelerationRate;
    }

    public double getAccelerationPower(double targetPower) {
        return targetPower * accelerationRate;
    }

    public double getNumOfAccelerationSegments() {
        return 1 / accelerationRate;
    }

    public double getAccelerationProportion() {
        return accelerationProportion;
    }

    public double getAccelerationRate() {
        return accelerationRate;
    }

    public String getDebug(){
        return debug;
    }

    private String arrayToString(double[] input){
        String output = "";
        for(int i = 0; i < input.length; i++)
            output += i + ">" + input[i] + " ";
        return output;
    }

    private String arrayTargetDistanceToString(ConstraintsAndConstants constants, double[] input){
        String output = "";
        for(int i = 0; i < input.length; i++)
            output += i + ">" + encoderCountsPerAcceleration(constants.wheelInchesToEncoderTicks(input[i])) + " ";
        return output;
    }
}
