package org.firstinspires.ftc.teamcode._Libs;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * library of utility classes supporting sensor inputs
 * Created by phanau on 1/1/16.
 */

public class SensorLib {

    
    public static class Utils {

        // wrap given angle into range (-180 .. +180)
        public static float wrapAngle(float angle) {
            while (angle > 180)
                angle -= 360;
            while (angle < -180)
                angle += 360;
            return angle;
        }

        // wrap given angle into range (-180 .. +180)
        public static double wrapAngle(double angle) {
            while (angle > 180)
                angle -= 360;
            while (angle < -180)
                angle += 360;
            return angle;
        }

    }

    public static class PID {

        private float mPrevError = 0;
        private float mIntegral = 0;
        float mKp = 0;
        float mKi = 0;
        float mKd = 0;
        float mKiCutoff = 0;     // max error value for which we integrate error over time

        public PID(float Kp, float Ki, float Kd, float KiCutoff) {
            setK(Kp, Ki, Kd, KiCutoff);
        }

        // set the parameters of the filter
        public void setK(float Kp, float Ki, float Kd, float KiCutoff)
        {
            // set filter coefficients
            mKp = Kp;
            mKi = Ki;
            mKd = Kd;

            // set threshold for errors we integrate -- integrating large errors causes instability
            mKiCutoff = KiCutoff;
        }

        // run one cycle of the PID filter given current error and delta-time since the previous call
        public float loop(float error, float dt) {
            if (Math.abs(error) < mKiCutoff)      // only integrate small errors to avoid wild over-correction
                mIntegral += error*dt;
            float derivative = (dt > 0) ? (error - mPrevError)/dt : 0;
            float output = mKp*error + mKi*mIntegral + mKd*derivative;
            mPrevError = error;
            return output;
        }

    }

    // handle interactive adjustment of PID parameters using controller
    public static class PIDAdjuster {
        OpMode mOpMode;
        PID mPID;
        Gamepad mGamepad;

        public PIDAdjuster(OpMode opmode, PID pid, Gamepad gamepad) {
            mOpMode = opmode;
            mPID = pid;
            mGamepad = gamepad;
        }

        public boolean loop() {
            // adjust PID parameters by joystick inputs - use thresholds to reduce cross-axis changes
            mPID.mKp -= (Math.abs(mGamepad.left_stick_y) < 0.1f ? 0 : mGamepad.left_stick_y * 0.0001f);
            mPID.mKi -= (Math.abs(mGamepad.right_stick_y) < 0.1f ? 0 : mGamepad.right_stick_y * 0.0001f);
            mPID.mKd += (Math.abs(mGamepad.left_stick_x) < 0.1f ? 0 : mGamepad.left_stick_x* 0.0001f);
            mPID.mKiCutoff += (Math.abs(mGamepad.right_stick_x) < 0.1f ? 0 : mGamepad.right_stick_x * 0.01f);

            // log updated values to the operator's console
            if (mOpMode != null) {
                mOpMode.telemetry.addData("Kp = ", mPID.mKp);
                mOpMode.telemetry.addData("Ki = ", mPID.mKi);
                mOpMode.telemetry.addData("Kd = ", mPID.mKd);
                mOpMode.telemetry.addData("KiCutoff = ", mPID.mKiCutoff);
            }
            return true;
        }
    }

    // class that tries to correct systemic errors in ModernRoboticsI2cGyro output
    public static class CorrectedGyro implements HeadingSensor {

        GyroSensor mGyro;            // the underlying physical Gyro
        float mCorrection = (360.0f/376.0f);    // default correction factor = ~16 degrees per revolution
        float mHeadingOffset = 0;

        public CorrectedGyro(GyroSensor gyro)
        {
            // remember the physical gyro we're using
            mGyro = gyro;
        }

        public void calibrate()
        {
            // start a calibration sequence on the gyro and wait for it to finish
            mGyro.calibrate();
            while (mGyro.isCalibrating()) {
                try {Thread.sleep(100);}
                catch (Exception e) {}
            }

            // reset the on-board z-axis integrator and wait for it to zero
            mGyro.resetZAxisIntegrator();
            while (mGyro.getHeading() != 0);
        }

        // override the default correction factor if your gyro is different
        public void setCorrection(float corr)
        {
            mCorrection = corr;
        }

        // implements HeadingSensor interface
        public float getHeading()
        {
            // since the physical gyro appears to have a small (~5%) error in the angles it reports,
            // we scale the cumulative integrated Z reported by the gyro and use that integrated Z to compute "heading".

            float intZ = mGyro.getHeading();     // our convention is positive CCW (right hand rule) -- need to check actual output <<< ???
            intZ *= mCorrection;                    // apply corrective scaling factor (empirically derived by testing)
            float heading = Utils.wrapAngle(intZ + mHeadingOffset);  // add angle offset and wrap to [-180..180) range

            return heading;         // unlike Gyro interface, we return this as float, not int
        }

        public boolean haveHeading()
        {
            return !mGyro.isCalibrating();  // data is always available from this device once it's calibrated
        }

        public void setHeadingOffset(float offset) { mHeadingOffset = offset; }

        public void stop()
        {
            mGyro.close();
        }
    }

    // class that tries to correct systemic errors in ModernRoboticsI2cGyro output
    public static class CorrectedMRGyro implements HeadingSensor {

        ModernRoboticsI2cGyro mGyro;            // the underlying physical MR Gyro
        float mCorrection = (360.0f/376.0f);    // default correction factor = ~16 degrees per revolution
        float mHeadingOffset = 0;

        public CorrectedMRGyro(ModernRoboticsI2cGyro gyro)
        {
            // remember the physical gyro we're using
            mGyro = gyro;
        }

        public void calibrate()
        {
            // start a calibration sequence on the gyro and wait for it to finish
            mGyro.calibrate();
            while (mGyro.isCalibrating()) {
                try {Thread.sleep(100);}
                catch (Exception e) {}
            }

            // reset the on-board z-axis integrator and wait for it to zero
            mGyro.resetZAxisIntegrator();
            while (mGyro.getIntegratedZValue() != 0);
        }

        // override the default correction factor if your gyro is different
        public void setCorrection(float corr)
        {
            mCorrection = corr;
        }

        public float getIntegratedZValue()
        {
            // return the raw (uncorrected) integrated Z value from the underlying physical gyro

            /* from MR Sensor Documentation:
            Integrated Z Value:
            The integrated gyro Z value returns the current value obtained by integrating the Z axis rate value, adjusted by the Z axis offset continuously.
            This value can also be used as a signed heading value where CW is in the positive direction and CCW is in the negative direction.
            */

            return mGyro.getIntegratedZValue();
        }

        // implements HeadingSensor interface
        public float getHeading()
        {
            // since the physical gyro appears to have a small (~5%) error in the angles it reports,
            // we scale the cumulative integrated Z reported by the gyro and use that integrated Z to compute "heading".

            float intZ = getIntegratedZValue();     // our convention is positive CCW (right hand rule) -- need to check actual output <<< ???
            intZ *= mCorrection;                    // apply corrective scaling factor (empirically derived by testing)
            float heading = Utils.wrapAngle(intZ + mHeadingOffset);  // add angle offset and wrap to [-180..180) range

            return heading;         // unlike Gyro interface, we return this as float, not int
        }

        public boolean haveHeading()
        {
            return !mGyro.isCalibrating();  // data is always available from this device once it's calibrated
        }

        public void setHeadingOffset(float offset) { mHeadingOffset = offset; }

        public void stop()
        {
            // mGyro.close();           // this causes big problems with REV hub ???
        }
    }

    // class that wraps UltrasonicSensor in a DistanceSensor interface
    public static class UltrasonicDistanceSensor implements DistanceSensor {

        private UltrasonicSensor mSensor;

        public UltrasonicDistanceSensor(UltrasonicSensor us)
        {
            mSensor = us;
        }

        public boolean haveDistance()
        {
            return mSensor.getUltrasonicLevel() < 255;
        }

        public float getDistance()
        {
            final float UUtoMM = 0.3f*25.4f;    // "Ultrasonic Units" appear to be ~0.3"
            return (float)mSensor.getUltrasonicLevel() * UUtoMM;
        }
    }

    // class that integrates movement and direction data to estimate a position on the field
    public static class PositionIntegrator {

        Position mPosition;

        public PositionIntegrator() {
            mPosition = new Position(DistanceUnit.INCH, 0,0,0,0);
        }

        public PositionIntegrator(double x, double y) {     // initial position
            mPosition = new Position(DistanceUnit.INCH, x, y, 0, 0);
        }

        public PositionIntegrator(Position position) {
            mPosition = position;
        }

        // move the position the given distance along the given bearing -
        // bearing is absolute in degrees with zero being along the field Y-axis, positive CCW
        public void move(double distance, double bearing) {
            double a = Math.toRadians(bearing);
            mPosition.x -= distance * Math.sin(a);
            mPosition.y += distance * Math.cos(a);
        }

        // move the position the given distances in vehicle X (right) and Y (forward) on the given bearing -
        // bearing is absolute in degrees with zero being along the field Y-axis, positive CCW
        public void move(double dx, double dy, double bearing) {
            double a = Math.toRadians(bearing);
            mPosition.x += dx * Math.cos(a) - dy * Math.sin(a);
            mPosition.y += dx * Math.sin(a) + dy * Math.cos(a);
        }

        // get the current position
        public double getX() { return mPosition.x; }
        public double getY() { return mPosition.y; }
        public Position getPosition() { return mPosition; }
    }
}