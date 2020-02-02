/**
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.corelib.drive.mecanum;

import android.util.Log;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.quickstart.util.AxesSigns;
import com.acmerobotics.roadrunner.quickstart.util.BNO055IMUUtil;
import com.acmerobotics.roadrunner.quickstart.util.LynxOptimizedI2cFactory;
import com.google.common.base.Optional;
import com.hfrobots.tnt.corelib.util.LynxModuleUtil;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import lombok.NonNull;

import static com.acmerobotics.roadrunner.quickstart.drive.DriveConstants.encoderTicksToInches;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class RoadRunnerMecanumDriveREVOptimized extends RoadRunnerMecanumDriveBase {
    private ExpansionHubEx hub;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    public RoadRunnerMecanumDriveREVOptimized(final DriveConstants driveConstants,
                                              SimplerHardwareMap hardwareMap,
                                              boolean needImu) {
        this(driveConstants, hardwareMap, needImu, Optional.<AxesOrder>absent(), Optional.<AxesSigns>absent());
    }

    public RoadRunnerMecanumDriveREVOptimized(final DriveConstants driveConstants,
                                              SimplerHardwareMap hardwareMap,
                                              boolean needImu,
                                              Optional<AxesOrder> optionalAxesOrder,
                                              Optional<AxesSigns> optionalAxesSigns) {
        super(driveConstants);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        hub = hardwareMap.get(ExpansionHubEx.class, "Main Hub");

        if (needImu) {
            imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);

            if (optionalAxesOrder.isPresent() && optionalAxesSigns.isPresent()) {
                // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
                // upward (normal to the floor) using a command like the following:
                BNO055IMUUtil.remapAxes(imu, optionalAxesOrder.get(), optionalAxesSigns.get());
            }
        }

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFrontDriveMotor");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRearDriveMotor");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRearDriveMotor");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFrontDriveMotor");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            // TO-DONE: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TO-DONE: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        // setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ...);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        if (imu == null) {
            return 0;
        }

        Orientation orientation = imu.getAngularOrientation();


        double angle = orientation.firstAngle;
        Log.d("RR", "imu-angles:" + orientation.firstAngle + ", " + orientation.secondAngle + ", " + orientation.thirdAngle);
        Log.d( "RR", "imu axes order: " + orientation.axesOrder + " in " + orientation.angleUnit);
        return angle;
    }
}
