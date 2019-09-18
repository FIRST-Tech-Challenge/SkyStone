package org.firstinspires.ftc.teamcode._Libs;

//import android.support.annotation.ColorInt;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Created by phanau on 12/14/15.
 */

// a library of classes that support autonomous opmode programming
public class AutoLib {

    // the base class from which everything else derives.
    // each action in an autonomous sequence is a Step of some kind.
    // a Step may be simple (like run a Motor) or a composite of several Steps which
    // are either run sequentially or in parallel (see Sequences below).
    static public abstract class Step {

        int mLoopCount;     // keeps count of how many times loop() has been called on this Step

        protected Step() {
            mLoopCount = 0;
        }

        // returns true iff called from the first call to loop() on this Step
        public boolean firstLoopCall() {
            return (mLoopCount == 1);    // assume this is called AFTER super.loop()
        }

        // run the next time-slice of the Step; return true when the Step is completed
        public boolean loop() {
            mLoopCount++;       // increment the loop counter
            return false;
        }

        // reset the Step so it can be used repeatedly by teleop modes, too
        public void reset() {
            mLoopCount = 0;
        }

        // get count of how many times loop() has been called on this step
        public int loopCount() { return mLoopCount; }

    }

    // ------------------ some implementations of Step aggregation constructs -------------------------

    // base class for Sequences that perform multiple Steps, either sequentially or concurrently
    static public abstract class Sequence extends Step {
        protected ArrayList<Step> mSteps;  // expandable array containing the Steps in the Sequence

        protected Sequence() {
            mSteps = new ArrayList<Step>(10);   // create the array with an initial capacity of 10
        }

        // add a Step to the end of the Sequence
        public Step add(Step step) {
            mSteps.add(step);
            return this;        // allows daisy-chaining of calls
        }

        // add a Step to the beginning of the Sequence - used for a control steps that
        // needs to run BEFORE the e.g. motor steps it controls
        public Step preAdd(Step step) {
            mSteps.add(0, step);
            return this;
        }

        // run the next time-slice of the Sequence; return true when the Sequence is completed.
        public boolean loop() {
            super.loop();
            return false;
        }

        // reset all the Steps in the Sequence
        public void reset() {
            super.reset();
            for (Step s : mSteps)
                s.reset();
        }
    }

    // a Sequence that performs its constituent Steps sequentially
    static public class LinearSequence extends Sequence {
        int mIndex;     // index of currently active Step

        public LinearSequence() {
            mIndex = 0;     // start at the beginning
        }

        // run the current Step of the Sequence until it completes, then the next Step and
        // the next, etc., etc. until the last Step completes, at which point the Sequence
        // returns complete.
        public boolean loop() {
            super.loop();
            if (mIndex < mSteps.size()) {       // if this Sequence is not completed ...
                if (mSteps.get(mIndex).loop())  // if this Step is complete, move to the next Step
                    mIndex++;
            }
            return (mIndex >= mSteps.size());   // return true when last Step completes
        }

        // reset this Sequence to its beginning
        public void reset() {
            super.reset();
            mIndex = 0;
        }
    }

    // a Sequence that performs its constituent Steps concurrently
    static public class ConcurrentSequence extends Sequence {

        public ConcurrentSequence() {
        }

        // run all the Steps in the Sequence "concurrently" -- i.e. run the loop() function of
        // each of the Steps each time loop() is called. When ALL the Steps report that they
        // are done, then this Sequence is done.
        public boolean loop() {
            super.loop();
            boolean bDone = true;
            for (Step s : mSteps)
                bDone &= s.loop();      // "done" means ALL Steps are done
            return bDone;
        }

        // reset: let superclass take care of it
    }

    // a Step that implements an if-then construct --
    // if the "if" step returns true, execute the "then" step, else execute the "else" step.
    // note that the evaluation of the "if" clause happens EVERY TIME through the loop(), not just once,
    // so if you want only the first evaluation to control the behavior of future loop() calls
    // you need to "latch" the result in your "if" step.
    static public class IfThenStep extends Step {
        Step mIf;
        Step mThen;
        Step mElse;

        public IfThenStep(Step ifStep,  Step thenStep, Step elseStep) {
            mIf = ifStep;
            mThen = thenStep;
            mElse = elseStep;
        }

        public boolean loop() {
            if (mIf.loop())
                return mThen.loop();
            else
                return mElse.loop();
        }

        public void reset() {
            super.reset();
            mIf.reset();
            mThen.reset();
            mElse.reset();
        }
    }

    // a Step that implements an n-way switch construct --
    // you derive your Step from this one and override the caseTest() function to implement your switch logic.
    static public class SwitchStep extends Step {
        Step[] mCases;
        boolean mOnce;
        int mCase;

        public SwitchStep() {}

        public SwitchStep(Step[] cases, boolean once) {
            mCases = cases;
            mOnce = once;
            mCase = -1;     // undefined
        }

        // if "once" is true, we run this code only until it returns a non-negative case and remember the result thereafter.
        // otherwise, this code is run every time through the loop.
        int caseTest() {
            return -1;      // undefined
        }

        public boolean loop() {
            // evaluate the case to run -- either once (until we get a valid result) or every time
            if (mOnce & mCase<0 || !mOnce)
                mCase = caseTest();

            // run the indicated case iff we have a valid case value and return its result, else return false (not done)
            return mCase>=0 ? mCases[mCase].loop() : false;
        }

        public void reset() {
            super.reset();
            for (Step s : mCases )
                s.reset();
            mCase = -1;     // undefined
        }
    }

    static public interface ReturnInteger {
        public void setInteger(int value);
    }

    static public class SwitchStep2 extends SwitchStep implements ReturnInteger {
        Step mCaseStep;     // the Step that sets the case to run

        public SwitchStep2(Step caseStep, Step[] cases, boolean once) {
            super(cases, once);
            mCaseStep = caseStep;
        }

        // callback from caseStep to return and set the case value
        public void setInteger(int value) {
            mCase = value;
        }

        // run caseStep.loop() and return the value it sent back to us via setInteger.
        // until it does set a valid value, we will return -1 (undefined),
        // which causes the base class loop() function to do nothing.
        int caseTest() {
            mCaseStep.loop();
            return mCase;
        }
    }


    // a Step that implements a do-until construct --
    // repeat the "do" Step until the "until" Step returns done.
    // reset the "do" Step each time "until" Step says do it again.
    static public class DoUntilStep extends Step {
        Step mDo;
        Step mUntil;

        public DoUntilStep(Step doStep, Step untilStep) {
            mDo = doStep;
            mUntil = untilStep;
        }

        public boolean loop() {
            boolean done = false;
            if (mDo.loop()) {             // if do content is done ...
                if (mUntil.loop()) {      // and until test is done ...
                    done = true;          // we're done
                }
                else {
                    mDo.reset();          // reset the do content so we can do it again ...
                }
            }
            return done;
        }

        public void reset() {
            super.reset();
            mDo.reset();
            mUntil.reset();
        }
    }

    // a Step that waits until a given test Step has returned true for a given length of time.
    // e.g. if you want to turn to a given heading, you might wait until a "read gyro" step (like GyroTestHeadingStep)
    // has reported that you're within 3 degrees of a given heading for 1 second to make
    // sure you've actually settled at that heading and aren't just "passing through it".
    static public class WaitTimeTestStep extends Step {
        Step mTest;         // the "test" Step
        Timer mTimer;       // Timer for this Step

        public WaitTimeTestStep(Step test, double seconds) {
            mTest = test;
            mTimer = new Timer(seconds);
        }

        public boolean loop() {
            super.loop();

            // start the Timer on our first call
            if (firstLoopCall())
                mTimer.start();

            // if the test Step fails, restart the Timer; else let it keep running
            if (!mTest.loop())
                mTimer.start();

            // return true when time is exhausted --
            // i.e. when the Test has succeeded continuously for the given time
            return (mTimer.done());
        }
    }

    // ------------------ some implementations of primitive Steps ----------------------------

    // a simple Step that just logs its existence for a given number of loop() calls
    // really just for testing sequencer stuff, not for actual use on a robot.
    static public class LogCountStep extends Step {
        OpMode mOpMode;     // needed so we can log output
        String mName;       // name of the output field
        int mCount;         // current loop count of this Step
        int mCount0;     // original loop count

        public LogCountStep(OpMode opMode, String name, int loopCount) {
            mOpMode = opMode;
            mName = name;
            mCount = mCount0 = loopCount;
        }

        public boolean loop() {
            super.loop();

            // log some info about this Step
            if (mCount > 0) {
                mOpMode.telemetry.addData(mName, "count = " + mCount);
                mCount--;
            } else
                mOpMode.telemetry.addData(mName, "done");


            // return true when count is exhausted
            return (mCount <= 0);
        }

        // reset the Step so it can be used repeatedly by teleop modes, too
        public void reset() {
            super.reset();
            mCount = mCount0;
        }

    }

    // a simple Step that just logs its existence for a given length of time
    static public class LogTimeStep extends Step {
        OpMode mOpMode;     // needed so we can log output
        String mName;       // name of the output field
        Timer mTimer;       // Timer for this Step

        public LogTimeStep(OpMode opMode, String name, double seconds) {
            mOpMode = opMode;
            mName = name;
            mTimer = new Timer(seconds);
        }

        public boolean loop() {
            super.loop();

            // start the Timer on our first call
            if (firstLoopCall())
                mTimer.start();

            // log some info about this Step
            if (!mTimer.done())        // appears to cycle here at about 3ms/loop
                mOpMode.telemetry.addData(mName, "time = " + mTimer.remaining());
            else
                mOpMode.telemetry.addData(mName, "done");

            // return true when time is exhausted
            return (mTimer.done());
        }

    }

    // interface for setting the current power of either kind of MotorStep
    public interface SetPower {
        public void setPower(double power);
    }

    // a Step that runs a DcMotor at a given power, for a given time
    static public class TimedMotorStep extends Step implements SetPower {
        Timer mTimer;
        DcMotor mMotor;
        double mPower;
        boolean mStop;          // stop motor when count is reached

        public TimedMotorStep(DcMotor motor, double power, double seconds, boolean stop) {
            mMotor = motor;
            mPower = power;
            mTimer = new Timer(seconds);
            mStop = stop;
        }

        // for dynamic adjustment of power during the Step
        public void setPower(double power) {
            mPower = power;
        }

        public boolean loop() {
            super.loop();

            // start the Timer and start the motor on our first call
            if (firstLoopCall()) {
                mTimer.start();
                mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mMotor.setPower(mPower);
            }

            // run the motor at the requested power until the Timer runs out
            boolean done = mTimer.done();
            if (done && mStop)
                mMotor.setPower(0);
            else
                mMotor.setPower(mPower);        // update power in case it changed

            return done;
        }

    }

    // a Step that runs a DcMotor at a given power, for a given encoder count relative to start of this Step
    static public class EncoderMotorStep extends Step implements SetPower {
        DcMotor mMotor;    // motor to control
        double mPower;          // power level to use
        int mEncoderCount;      // incremental target encoder count
        boolean mStop;          // stop motor when count is reached

        public EncoderMotorStep(DcMotor motor, double power, int count, boolean stop) {
            mMotor = motor;
            mPower = power;
            mEncoderCount = count;
            mStop = stop;
        }

        // for dynamic adjustment of power during the Step
        public void setPower(double power) {
            mPower = power;
        }

        public boolean loop() {
            super.loop();

            boolean done = false;

            // we need a little state machine to make the encoders happy
            if (firstLoopCall()) {
                // set up the motor on our first call
                mMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mMotor.setTargetPosition(mMotor.getCurrentPosition() + mEncoderCount);  // count is RELATIVE to NOW
                mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mMotor.setPower(mPower);
            }

            // the rest of the time, just update power and check to see if we're done
            done = !mMotor.isBusy();
            if (done && mStop)
                mMotor.setPower(0);     // optionally stop motor when target reached
            else
                mMotor.setPower(mPower);        // update power in case it changed

            return done;
        }

    }

    // a Step that drives a Servo to a given position
    // it would be nice if we got actual position info back from the servo, but that's not
    // how it works, so client sequences should either
    // * tell us how long it takes for the servo travel through its full range (0..1) by using the second constructor, or
    // * follow each ServoStep with a LogTimeStep or some other step that takes long enough for it to get where it's told to go.
    static public class ServoStep extends Step {
        Servo mServo;
        double mPosition;          // target position of servo
        Timer mTimer;              // Timer for this Step
        double mFullRangeTime;     // time (sec) it takes to move servo from 0..1 or v.v.

        // this constructor assumes client sequence is dealing with waiting for the servo to get where it's going
        public ServoStep(Servo servo, double position) {
            mServo = servo;
            mPosition = position;
            mTimer = null;
            mFullRangeTime = 0.0;   // servo is assumed to be instantaneous if not told otherwise
        }

        // this constructor waits for a time determined by the given full range motion time and the difference
        // between the commanded position of this step and the previous commanded position of the Servo object.
        public ServoStep(Servo servo, double position, double fullRangeTime) {
            mServo = servo;
            mPosition = position;
            mTimer = null;
            mFullRangeTime = fullRangeTime;   // assumed if not told otherwise
        }

        public boolean loop() {
            super.loop();

            if (firstLoopCall()) {
                // start a timer that estimates when the motion will complete
                // assuming servo has remembered the last position it was ordered to and
                // assuming servo takes mFullRangeTime to go through full range (0..1)
                double seconds = Double.isNaN(mServo.getPosition())     // uncommanded Servo may return NaN for position
                        ? 0.0           // in which case, we set our wait time to zero (i.e. this step completes immediately)
                        : Math.abs(mPosition-mServo.getPosition()) * mFullRangeTime;    // estimate wait time
                mTimer = new Timer(seconds);
                mTimer.start();

                // now tell the servo to go to the target position
                mServo.setPosition(mPosition);
            }

            // we're done when we've waited long enough for
            // the servo to (probably) get to the ordered position
            boolean done = mTimer.done();

            return done;
        }

    }


    // some utility functions

    // linear interpolation
    private static double lerp (double x, double x0, double x1, double y0, double y1)
    {
        return ((x-x0)/(x1-x0))*(y1-y0) + y0;
    }

    // return normalization factor that makes max magnitude of any argument the magnitude of f
    static public float normalize(float f, float a, float b)
    {
        float af = Math.abs(f);
        float m = Math.max(Math.abs(a), Math.abs(b));
        return (m > af) ? af/m : 1.0f;
    }
    static public float normalize(float a, float b) { return normalize(1.0f, a, b); }

    static public float normalize(float f, float a, float b, float c, float d)
    {
        float af = Math.abs(f);
        float m = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
        return (m > af) ? af/m : 1.0f;
    }
    static public float normalize(float a, float b, float c, float d) { return normalize(1.0f, a, b, c, d); }

    // return normalization factor that makes max magnitude of any argument 1
    static public double normalize(double f, double a, double b)
    {
        double af = Math.abs(f);
        double m = Math.max(Math.abs(a), Math.abs(b));
        return (m > af) ? af/m : 1.0;
    }
    static public double normalize(double a, double b) { return normalize(1.0, a, b); }

    static public double normalize(double f, double a, double b, double c, double d)
    {
        double af = Math.abs(f);
        double m = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
        return (m > af) ? af/m : 1.0;
    }
    static public double normalize(double a, double b, double c, double d) { return normalize(1.0, a, b, c, d); }

    static public float normalize(float f, float[] a)
    {
        float af = Math.abs(f);
        float m = 0;
        for (float x : a)
            m = Math.max(m, Math.abs(x));
        return (m > af) ? af/m : 1.0f;
    }
    static public float normalize(float[] a) { return normalize(1.0f, a); }

    static public double normalize(double f, double[] a)
    {
        double af = Math.abs(f);
        double m = 0;
        for (double x : a)
            m = Math.max(m, Math.abs(x));
        return (m > af) ? af/m : 1.0;
    }
    static public double normalize(double[] a) { return normalize(1.0, a); }


    // some Steps that use various sensor input to control other Steps

    // interface for setting the current power of either kind of MotorStep
    public interface SetMotorSteps {
        public void set(ArrayList<AutoLib.SetPower> motorsteps);
    }

    static public abstract class MotorGuideStep extends AutoLib.Step implements SetMotorSteps {
        public void set(ArrayList<AutoLib.SetPower> motorsteps){}
    }

    // update target direction, heading, and power --
    // used by interactive teleop modes to redirect a guide step from controller input
    // or e.g. by camera based guide steps to steer the robot to a target
    public interface SetDirectionHeadingPower {
        public void setDirection(float direction);              // absolute
        public void setRelativeDirection(float direction);      // relative to current orientation (heading)
        public void setHeading(float heading);
        public void setPower(float power);
        public void setMaxPower(float power);
    }


    // a Step that provides gyro-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps in order right ..., left ...
    // this step tries to keep the robot on the given course by adjusting the left vs. right motors to change the robot's heading.
    static public class GyroGuideStep extends MotorGuideStep implements SetDirectionHeadingPower {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private float mDirection;                           // absolute direction along which the robot should move (0 ahead; positive CCW)
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        private HeadingSensor mGyro;                        // sensor to use for heading information (e.g. Gyro or Vuforia)
        private SensorLib.PID mPid;                         // proportional–integral–derivative controller (PID controller)
        private double mPrevTime;                           // time of previous loop() call
        private ArrayList<SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...
        private float mMaxPower;                            // max allowed power including direction correction

        public GyroGuideStep(OpMode mode, float direction, HeadingSensor gyro, SensorLib.PID pid,
                             ArrayList<SetPower> motorsteps, float power)
        {
            mOpMode = mode;
            mDirection = direction;
            mGyro = gyro;
            if (pid != null)
                mPid = pid;     // client is supplying PID controller for correcting heading errors
            else {
                // construct a default PID controller for correcting heading errors
                final float Kp = 0.05f;        // degree heading proportional term correction per degree of deviation
                final float Ki = 0.02f;        // ... integrator term
                final float Kd = 0.0f;         // ... derivative term
                final float KiCutoff = 3.0f;   // maximum angle error for which we update integrator
                mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);
            }
            mMotorSteps = motorsteps;
            mPower = power;
            mMaxPower = power;
        }

        // set max allowed power including direction correction ---
        // e.g. to turn in place slowly, set power=0 and maxPower<1.0
        public void setMaxPower(float mp) {
            mMaxPower = mp;
        }

        // set motor control steps this step should control (assumes ctor called with null argument)
        public void set(ArrayList<AutoLib.SetPower> motorsteps)
        {
            mMotorSteps = motorsteps;
        }

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        // and by e.g. camera based guide steps to steer the robot to a target
        public void setDirection(float direction) { mDirection = direction; }
        public void setRelativeDirection(float direction) { mDirection = mGyro.getHeading() + direction; }
        public void setHeading(float heading) { mDirection = heading; } // heading == direction for this guide step
        public void setPower(float power) { mPower = power; }

        public boolean loop()
        {
            super.loop();

            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
            }

            float heading = mGyro.getHeading();     // get latest reading from direction sensor
            // convention is positive angles CCW, wrapping from 359-0

            float backwards = (mPower < 0) ? 180 : 0;       // if we're going backwards, reverse the gyro reading
            float error = SensorLib.Utils.wrapAngle(heading + backwards - mDirection);   // deviation from desired heading
            // deviations to left are positive, to right are negative

            // compute delta time since last call -- used for integration time of PID step
            double time = mOpMode.getRuntime();
            double dt = time - mPrevTime;
            mPrevTime = time;

            // feed error through PID to get motor power correction value
            float correction = -mPid.loop(error, (float)dt);

            // compute new right/left motor powers
            float rightPower = mPower + correction;
            float leftPower = mPower - correction;

            // normalize so neither has magnitude > maxPower
            float norm = normalize(mMaxPower, rightPower, leftPower);
            rightPower *= norm;
            leftPower *= norm;

            // set the motor powers -- handle both time-based and encoder-based motor Steps
            // assumed order is right motors followed by an equal number of left motors
            int i = 0;
            for (SetPower ms : mMotorSteps) {
                ms.setPower((i++ < mMotorSteps.size()/2) ? rightPower : leftPower);
            }

            // log some data
            if (mOpMode != null) {
                mOpMode.telemetry.addData("heading ", heading);
                mOpMode.telemetry.addData("error ", error);
                mOpMode.telemetry.addData("correction ", correction);
                mOpMode.telemetry.addData("mPower ", mPower);
                mOpMode.telemetry.addData("left power ", leftPower);
                mOpMode.telemetry.addData("right power ", rightPower);
            }

            // guidance step always returns "done" so the CS in which it is embedded completes when
            // all the motors it's controlling are done
            return true;
        }

    }

    // a Step that provides guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps in order right ..., left ...
    // this step tries to steer the robot by adjusting the left vs. right motors to change the robot's heading in response
    // to error inputs from some other sensor-based step. These are set through calls to any of setDirection, setRelativeDirection, or setHeading.
    // deviations to left are positive, to right are negative (right handed Yaw axis)
    static public class ErrorGuideStep extends MotorGuideStep implements SetDirectionHeadingPower {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private float mDirection;                           // current deviation from direction along which the robot should move (0 on course; positive CCW)
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        private SensorLib.PID mPid;                         // proportional–integral–derivative controller (PID controller)
        private double mPrevTime;                           // time of previous loop() call
        private ArrayList<SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...
        private float mMaxPower;                            // max allowed power including direction correction

        public ErrorGuideStep(OpMode mode, SensorLib.PID pid, ArrayList<SetPower> motorsteps, float power)
        {
            mOpMode = mode;
            mDirection = 0;     // initially no error
            if (pid != null)
                mPid = pid;     // client is supplying PID controller for correcting heading errors
            else {
                // construct a default PID controller for correcting heading errors
                final float Kp = 0.03f;        // degree heading proportional term correction per degree of deviation
                final float Ki = 0.005f;        // ... integrator term
                final float Kd = 0.0f;         // ... derivative term
                final float KiCutoff = 0.0f;   // maximum angle error for which we update integrator
                mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);
            }
            mMotorSteps = motorsteps;
            mPower = power;
            mMaxPower = power;
        }

        // set max allowed power including direction correction ---
        // e.g. to turn in place slowly, set power=0 and maxPower<1.0
        public void setMaxPower(float mp) {
            mMaxPower = mp;
        }

        // set motor control steps this step should control (assumes ctor called with null argument)
        public void set(ArrayList<AutoLib.SetPower> motorsteps)
        {
            mMotorSteps = motorsteps;
        }

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        // and by e.g. camera based guide steps to steer the robot to a target
        // convention: for consistency with gyros, headings are positive CCW, so here deviations to left are positive, to right are negative
        public void setDirection(float direction) { mDirection = direction; }
        public void setRelativeDirection(float direction) { mDirection = direction; }
        public void setHeading(float heading) { mDirection = heading; } // heading == direction for this guide step
        public void setPower(float power) { mPower = power; }

        public boolean loop()
        {
            super.loop();

            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
            }

            float error = mDirection;   // deviation from desired heading

            // compute delta time since last call -- used for integration time of PID step
            double time = mOpMode.getRuntime();
            double dt = time - mPrevTime;
            mPrevTime = time;

            // feed error through PID to get motor power correction value
            float correction = mPid.loop(error, (float)dt);

            // compute new right/left motor powers
            float rightPower = mPower - correction;
            float leftPower = mPower + correction;

            // normalize so neither has magnitude > maxPower
            float norm = normalize(mMaxPower, rightPower, leftPower);
            rightPower *= norm;
            leftPower *= norm;

            // set the motor powers -- handle both time-based and encoder-based motor Steps
            // assumed order is right motors followed by an equal number of left motors
            int i = 0;
            for (SetPower ms : mMotorSteps) {
                ms.setPower((i++ < mMotorSteps.size()/2) ? rightPower : leftPower);
            }

            // log some data
            if (mOpMode != null) {
                mOpMode.telemetry.addData("error ", mDirection);
                mOpMode.telemetry.addData("left power ", leftPower);
                mOpMode.telemetry.addData("right power ", rightPower);
            }

            // guidance step always returns "done" so the CS in which it is embedded completes when
            // all the motors it's controlling are done
            return true;
        }

    }

    // dummy drive step for debug mode where we don't have motors or gyros
    static public class MotorLogStep extends AutoLib.MotorGuideStep implements AutoLib.SetDirectionHeadingPower {
        OpMode mOpMode;

        public MotorLogStep(OpMode opmode) {
            mOpMode = opmode;
        }

        public void setDirection(float direction)              // absolute
        {
            mOpMode.telemetry.addData("setDirection", direction);
        }
        public void setRelativeDirection(float direction)      // relative to current orientation (heading)
        {
            mOpMode.telemetry.addData("setRelativeDirection", direction);
        }
        public void setHeading(float heading)
        {
            mOpMode.telemetry.addData("setHeading", heading);
        }
        public void setPower(float power)
        {
            mOpMode.telemetry.addData("setPower", power);
        }
        public void setMaxPower(float power)
        {
            mOpMode.telemetry.addData("setMaxPower", power);
        }

        public boolean loop() {
            return false;
        }
    }

    // a Step that waits for valid location and heading data to be available --- e.g from Vuforia --
    // when added to either a dead reckoning or gyro-based movement step, it can be used to end that step
    // when we're close enough to the targets for Vuforia to start being used. The base step should be
    // of "zero-length" -- i.e. it should always be "done" so the composite step will be "done" as soon as
    // this step detects valid location and heading data.
    static public class LocationHeadingWaitStep extends Step {

        private LocationSensor mLocSensor;                  // sensor to use for field location information (e.g. Vuforia)
        private HeadingSensor mYawSensor;                   // sensor to use for robot orientation on field (may be same as LocationSensor)

        public LocationHeadingWaitStep(LocationSensor locSensor, HeadingSensor yawSensor)
        {
            mLocSensor = locSensor;
            mYawSensor = yawSensor;
        }

        public boolean loop()
        {
            super.loop();

            boolean bDone = true;
            if (mLocSensor != null)
                bDone &= mLocSensor.haveLocation();
            if (mYawSensor != null)
                bDone &= mYawSensor.haveHeading();
            return bDone;
        }
    }


    // a Step that returns true when the given HeadingSensor settles at a heading within some tolerance of a desired heading.
    static public class GyroTestHeadingStep extends Step {
        HeadingSensor mSensor;
        double mHeading;
        double mTolerance;
        int mInTolCount, mReqTolCount;
        Timer mTimer;       // Timer for this Step

        public GyroTestHeadingStep(HeadingSensor sensor, double heading, double tol, int count, float timeout){
            mSensor = sensor;
            mHeading = heading;
            mTolerance = tol;
            mReqTolCount = count;
            mInTolCount = 0;
            mTimer = new Timer(timeout);
        }

        public boolean loop() {
            super.loop();

            // start the Timer on our first call
            if (firstLoopCall())
                mTimer.start();

            // can't do this forever -- time out if timer expires
            if (mTimer.done())        // appears to cycle here at about 3ms/loop
                return true;            // we're done whether or not we were ever within tolerance

            if (mSensor.haveHeading()) {
                if (Math.abs(SensorLib.Utils.wrapAngle(mSensor.getHeading() - mHeading)) < mTolerance)
                    mInTolCount++;
                else
                    mInTolCount = 0;    // reset
                return (mInTolCount >= mReqTolCount);
            }
            else
                return false;
        }
    }

    // a Step that stops the given set of motor control steps and terminates
    // when the given DistanceSensor reports less than a given distance (in mm).
    // pass in an empty set of motors to just do the test (e.g. for use with WaitTimeTestStep).
    static public class DistanceSensorGuideStep extends Step {

        private OpMode mOpMode;                     // for telemetry output (optional)
        private DistanceSensor mDistanceSensor;     // distance sensor to read
        private float mDistance;                    // stopping distance
        private ArrayList<SetPower> mSteps;         // motors to stop at given distance

        public DistanceSensorGuideStep(OpMode opmode, DistanceSensor ds, float distance, ArrayList<SetPower> steps)
        {
            mOpMode = opmode;
            mDistanceSensor = ds;
            mDistance = distance;
            mSteps = steps;
        }

        public boolean loop()
        {
            super.loop();

            boolean have = mDistanceSensor.haveDistance();
            float dist = mDistanceSensor.getDistance();
            boolean done = have && (dist < mDistance);
            if (done)
                for (SetPower step : mSteps)
                    step.setPower(0);
            if (mOpMode != null) {
                mOpMode.telemetry.addData("DSGS: ", "have = %b  dist(mm) = %2.1f  (in) = %2.1f  done = %b", have, dist, dist/25.4f, done);
            }
            return done;
        }
    }

    // a generic Step that uses a MotorGuideStep to steer the robot while driving along a given path
    // until the terminatorStep tells us that we're there, thereby terminating this step.
    static public class GuidedTerminatedDriveStep extends AutoLib.ConcurrentSequence {

        public GuidedTerminatedDriveStep(OpMode mode, AutoLib.MotorGuideStep guideStep, AutoLib.MotorGuideStep terminatorStep, DcMotor[] motors)
        {
            // add a concurrent Step to control each motor
            ArrayList<AutoLib.SetPower> steps = new ArrayList<AutoLib.SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    AutoLib.TimedMotorStep step = new AutoLib.TimedMotorStep(em, 0, 0, false);
                    // the guide or terminator Step will stop the motors and complete the sequence
                    this.add(step);
                    steps.add(step);
                }

            // if there's a separate terminator step, tell it about the motor steps and add it to the sequence
            if (terminatorStep != null) {
                terminatorStep.set(steps);
                this.preAdd(terminatorStep);
            }

            // tell the guideStep about the motor Steps it should control
            guideStep.set(steps);

            // add a concurrent Step to control the motor steps based on sensor (gyro, camera, etc.) input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            // and BEFORE the terminatorStep might try to turn the motors off.
            this.preAdd(guideStep);
        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

    }


    // a Step that provides gyro-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // driving "squirrely wheels" that can move sideways by differential turning of front vs. back wheels.
    // assumes 4 concurrent drive motor steps in order right front, right back, left front, left back.
    // this step tries to maintain the robot's absolute orientation (heading) given by the gyro by adjusting the left vs. right motors
    // while the front vs. back power is adjusted to translate in the desired absolute direction.
    static public class SquirrelyGyroGuideStep extends AutoLib.MotorGuideStep implements SetDirectionHeadingPower {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private float mDirection;                           // absolute direction along which the robot should move (0 ahead; positive CCW)
        private float mHeading;                             // absolute orientation the robot should maintain while moving
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        private HeadingSensor mGyro;                        // sensor to use for heading information (e.g. Gyro or Vuforia)
        private SensorLib.PID mPid;                         // proportional–integral–derivative controller (PID controller)
        private double mPrevTime;                           // time of previous loop() call
        private ArrayList<AutoLib.SetPower> mMotorSteps;    // the motor steps we're guiding - assumed order is fr, br, fl, bl
        private float mMaxPower;                            // max allowed power including direction correction

        public SquirrelyGyroGuideStep(OpMode mode, float direction, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                      ArrayList<AutoLib.SetPower> motorsteps, float power) {
            mOpMode = mode;
            mDirection = direction;
            mHeading = heading;
            mGyro = gyro;
            if (pid != null)
                mPid = pid;     // client is supplying PID controller for correcting heading errors
            else {
                // construct a default PID controller for correcting heading errors
                final float Kp = 0.05f;        // degree heading proportional term correction per degree of deviation
                final float Ki = 0.02f;        // ... integrator term
                final float Kd = 0.0f;         // ... derivative term
                final float KiCutoff = 3.0f;   // maximum angle error for which we update integrator
                mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);
            }
            mMotorSteps = motorsteps;
            mPower = mMaxPower = power;
        }

        // set max allowed power including direction correction ---
        // e.g. to turn in place slowly, set power=0 and maxPower<1.0
        public void setMaxPower(float mp) {
            mMaxPower = mp;
        }

        // set motor control steps this step should control (assumes ctor called with null argument)
        public void set(ArrayList<AutoLib.SetPower> motorsteps)
        {
            mMotorSteps = motorsteps;
        }

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        // and by e.g. camera based guide steps to steer the robot to a target
        public void setDirection(float direction) { mDirection = direction; }
        public void setRelativeDirection(float direction) { mDirection = mGyro.getHeading() + direction; }
        public void setHeading(float heading) { mHeading = heading; }
        public void setPower(float power) { mPower = power; }

        public boolean loop()
        {
            super.loop();

            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
            }

            final float heading = mGyro.getHeading();     // get latest reading from direction sensor
            // convention is positive angles CCW, wrapping from 359-0

            final float error = SensorLib.Utils.wrapAngle(heading - mHeading);   // deviation from desired heading
            // deviations to left are positive, to right are negative

            // compute delta time since last call -- used for integration time of PID step
            final double time = mOpMode.getRuntime();
            final double dt = time - mPrevTime;
            mPrevTime = time;

            // feed error through PID to get motor power value for heading correction
            float hdCorr = mPid.loop(error, (float) dt);

            // relative direction we want to move is difference between requested absolute direction and CURRENT orientation
            float relDir = SensorLib.Utils.wrapAngle(mDirection - heading);

            // calculate relative front/back motor powers for fancy wheels to move us in requested relative direction
            AutoLib.MotorPowers mp = AutoLib.GetSquirrelyWheelMotorPowers(relDir);

            // calculate powers of the 4 motors
            double pFR = mp.Front() * mPower - hdCorr;
            double pBR = mp.Back() * mPower - hdCorr;
            double pFL = mp.Front() * mPower + hdCorr;
            double pBL = mp.Back() * mPower + hdCorr;

            // normalize powers so none has magnitude > maxPower
            double norm = normalize(mMaxPower, pFR, pBR, pFL, pBL);
            pFR *= norm;  pBR *= norm;  pFL *= norm;  pBL *= norm;

            // set the powers
            mMotorSteps.get(0).setPower(pFR);   // fr
            mMotorSteps.get(1).setPower(pBR);   // br
            mMotorSteps.get(2).setPower(pFL);   // fl
            mMotorSteps.get(3).setPower(pBL);   // bl

            // log some data
            if (mOpMode != null) {
                mOpMode.telemetry.addData("heading ", heading);
                mOpMode.telemetry.addData("front power ", mp.Front());
                mOpMode.telemetry.addData("back power ", mp.Back());
            }

            // guidance step always returns "done" so the CS in which it is embedded completes when
            // all the motors it's controlling are done
            return true;
        }

    }


    // some Steps that combine various motor driving Steps with guide Steps that control them

    // a Step that uses gyro input to drive along a given course for a given time.
    // uses a GyroGuideStep to adjust power to 2 or 4 motors.
    // assumes a robot with up to 4 drive motors in assumed order right motors, left motors
    static public class AzimuthTimedDriveStep extends ConcurrentSequence implements SetDirectionHeadingPower {

        public AzimuthTimedDriveStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                     DcMotor motors[], float power, float maxPower, float time, boolean stop)
        {
            this(mode, heading, gyro, pid, motors, power, time, stop);
            this.setMaxPower(maxPower);
        }

        public AzimuthTimedDriveStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                     DcMotor motors[], float power, float time, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, time, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target heading, and power --
        // used e.g. by interactive teleop modes to redirect the step from controller input
        public void setDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setDirection(direction); }
        public void setRelativeDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setRelativeDirection(direction); }
        public void setHeading(float heading) { ((GyroGuideStep)mSteps.get(0)).setHeading(heading); }
        public void setPower(float power) { ((GyroGuideStep)mSteps.get(0)).setPower(power); }
        public void setMaxPower(float power) { ((GyroGuideStep)mSteps.get(0)).setMaxPower(power); }
    }

    // a Step that uses gyro input to drive along a given course for a given distance given by motor encoders.
    // uses a GyroGuideStep to adjust power to 2 or 4 motors.
    // assumes a robot with up to 4 drive motors in assumed order right motors, left motors
    static public class AzimuthCountedDriveStep extends ConcurrentSequence implements SetDirectionHeadingPower {

        public AzimuthCountedDriveStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                     DcMotor motors[], float power, float maxPower, int count, boolean stop)
        {
            this(mode, heading, gyro, pid, motors, power, count, stop);
            this.setMaxPower(maxPower);
        }

        public AzimuthCountedDriveStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                       DcMotor motors[], float power, int count, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    EncoderMotorStep step = new EncoderMotorStep(em, power, count, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used e.g. by interactive teleop modes to redirect the step from controller input
        public void setDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setDirection(direction); }
        public void setRelativeDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setRelativeDirection(direction); }
        public void setHeading(float heading) { ((GyroGuideStep)mSteps.get(0)).setHeading(heading); }
        public void setPower(float power) { ((GyroGuideStep)mSteps.get(0)).setPower(power); }
        public void setMaxPower(float power) { ((GyroGuideStep)mSteps.get(0)).setMaxPower(power); }
    }

    // a Step that uses gyro input to drive along a given course until the given DistanceSensor
    // reports that we are within a given distance of some target.
    // uses a GyroGuideStep to adjust power to 2 or 4 motors.
    // assumes a robot with up to 4 drive motors in assumed order right motors, left motors
    static public class AzimuthDistanceDriveStep extends ConcurrentSequence implements SetDirectionHeadingPower {

        public AzimuthDistanceDriveStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                     DcMotor motors[], float power, DistanceSensor ds, float distance)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);
                    // the DistanceSensorGuideStep will stop the motors and complete the sequence
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on DistanceSensor input
            // put it 2nd in the list so it can set the power of the motors to zero BEFORE their steps run
            this.preAdd(new DistanceSensorGuideStep(mode, ds, distance, steps));

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            // and BEFORE the DistanceSensorGuideStep might try to turn the motors off.
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used e.g. by interactive teleop modes to redirect the step from controller input
        public void setDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setDirection(direction); }
        public void setRelativeDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setRelativeDirection(direction); }
        public void setHeading(float heading) { ((GyroGuideStep)mSteps.get(0)).setHeading(heading); }
        public void setPower(float power) { ((GyroGuideStep)mSteps.get(0)).setPower(power); }
        public void setMaxPower(float power) { ((GyroGuideStep)mSteps.get(0)).setMaxPower(power); }
    }

    // a Step that turns in place - just shorthand for a special case of its base class.
    // turn in place to given heading using given gyro sensor, waiting for given time.
    static public class AzimuthTimedTurnStep extends AzimuthTimedDriveStep {

        public AzimuthTimedTurnStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                    DcMotor motors[], float power, float time, boolean stop)
        {
            super(mode, heading, gyro, pid, motors, 0, power, time, stop);
        }

    }

    // a Step that turns in place -
    // turn in place to given heading using given gyro sensor, waiting for given time.
    static public class AzimuthTolerancedTurnStep extends ConcurrentSequence implements SetDirectionHeadingPower {

        public AzimuthTolerancedTurnStep(OpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                    DcMotor motors[], float power, float tol, float timeout)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to determine when we're done (close enough).
            this.preAdd(new GyroTestHeadingStep(gyro, heading, tol, 10, timeout));

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, 0));

            // to turn in place, we set the power to zero (above) and the max power of the GyroGuideStep to the given power
            setMaxPower(power);
        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used e.g. by interactive teleop modes to redirect the step from controller input
        public void setDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setDirection(direction); }
        public void setRelativeDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setRelativeDirection(direction); }
        public void setHeading(float heading) { ((GyroGuideStep)mSteps.get(0)).setHeading(heading); }
        public void setPower(float power) { ((GyroGuideStep)mSteps.get(0)).setPower(power); }
        public void setMaxPower(float power) { ((GyroGuideStep)mSteps.get(0)).setMaxPower(power); }

    }


    // a Step that uses gyro input to stabilize the robot orientation while driving along a given absolute heading
    // using squirrely wheels, for a given time.
    // uses a SquirrelyGyroGuideStep to adjust power to 4 motors in assumed order fr, br, fl, bl
    static public class SquirrelyGyroTimedDriveStep extends AutoLib.ConcurrentSequence implements SetDirectionHeadingPower {

        public SquirrelyGyroTimedDriveStep(OpMode mode, float direction, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                              DcMotor motors[], float power, float time, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<AutoLib.SetPower> steps = new ArrayList<AutoLib.SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, 0, time, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            this.preAdd(new SquirrelyGyroGuideStep(mode, direction, heading, gyro, pid, steps, power));
        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used e.g. by interactive teleop modes to redirect the step from controller input
        public void setDirection(float direction) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setDirection(direction); }
        public void setRelativeDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setRelativeDirection(direction); }
        public void setHeading(float heading) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setHeading(heading); }
        public void setPower(float power) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setPower(power); }
        public void setMaxPower(float power) { ((GyroGuideStep)mSteps.get(0)).setMaxPower(power); }
    }

    // a Step that uses gyro input to stabilize the robot orientation while driving along a given absolute heading
    // using squirrely wheels, for a given number of motor counts.
    // uses a SquirrelyGyroGuideStep to adjust power to 4 motors in assumed order fr, br, fl, bl
    static public class SquirrelyGyroCountedDriveStep extends ConcurrentSequence implements SetDirectionHeadingPower{

        public SquirrelyGyroCountedDriveStep(OpMode mode, float direction, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                       DcMotor motors[], float power, int count, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    EncoderMotorStep step = new EncoderMotorStep(em, power, count, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new SquirrelyGyroGuideStep(mode, direction, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used e.g. by interactive teleop modes to redirect the step from controller input
        public void setDirection(float direction) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setDirection(direction); }
        public void setRelativeDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setRelativeDirection(direction); }
        public void setHeading(float heading) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setHeading(heading); }
        public void setPower(float power) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setPower(power); }
        public void setMaxPower(float power) { ((GyroGuideStep)mSteps.get(0)).setMaxPower(power); }
    }

    // a Step that uses gyro input to stabilize the robot orientation while driving along a given relative heading
    // using squirrely wheels, until a DistanceSensor reports we're within a given distance of something.
    // uses a SquirrelyGyroGuideStep to adjust power to 4 motors in assumed order fr, br, fl, bl
    // and a DistanceSensorGuideStep to determine when to stop.
    static public class SquirrelyGyroDistanceDriveStep extends ConcurrentSequence implements SetDirectionHeadingPower {

        public SquirrelyGyroDistanceDriveStep(OpMode mode, float direction, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                              DcMotor motors[], float power, DistanceSensor ds, float distance)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);
                    // the DistanceSensorGuideStep will stop the motors and complete the sequence
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on DistanceSensor input
            // put it 2nd in the list so it can set the power of the motors to zero BEFORE their steps run
            this.preAdd(new DistanceSensorGuideStep(mode, ds, distance, steps));

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            // and BEFORE the DistanceSensorGuideStep might try to turn the motors off.
            this.preAdd(new SquirrelyGyroGuideStep(mode, direction, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used e.g. by interactive teleop modes to redirect the step from controller input
        public void setDirection(float direction) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setDirection(direction); }
        public void setRelativeDirection(float direction) { ((GyroGuideStep)mSteps.get(0)).setRelativeDirection(direction); }
        public void setHeading(float heading) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setHeading(heading); }
        public void setPower(float power) { ((SquirrelyGyroGuideStep)mSteps.get(0)).setPower(power); }
        public void setMaxPower(float power) { ((GyroGuideStep)mSteps.get(0)).setMaxPower(power); }
    }


    // some convenience utility classes for common operations

    // a Sequence that moves an up-to-four-motor robot in a straight line with given power for given time
    static public class MoveByTimeStep extends ConcurrentSequence {

        public MoveByTimeStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double power, double seconds, boolean stop)
        {
            if (fr != null)
                this.add(new TimedMotorStep(fr, power, seconds, stop));
            if (br != null)
                this.add(new TimedMotorStep(br, power, seconds, stop));
            if (fl != null)
                this.add(new TimedMotorStep(fl, power, seconds, stop));
            if (bl != null)
                this.add(new TimedMotorStep(bl, power, seconds, stop));
        }

        public MoveByTimeStep(DcMotor motors[], double power, double seconds, boolean stop)
        {
            for (DcMotor em : motors)
                if (em != null)
                    this.add(new TimedMotorStep(em, power, seconds, stop));
        }

    }


    // a Sequence that turns an up-to-four-motor robot by applying the given right and left powers for given time
    static public class TurnByTimeStep extends ConcurrentSequence {

        public TurnByTimeStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double rightPower, double leftPower, double seconds, boolean stop)
        {
            if (fr != null)
                this.add(new TimedMotorStep(fr, rightPower, seconds, stop));
            if (br != null)
                this.add(new TimedMotorStep(br, rightPower, seconds, stop));
            if (fl != null)
                this.add(new TimedMotorStep(fl, leftPower, seconds, stop));
            if (bl != null)
                this.add(new TimedMotorStep(bl, leftPower, seconds, stop));
        }

    }


    // a Sequence that moves an up-to-four-motor robot in a straight line with given power for given encoder count
    static public class MoveByEncoderStep extends ConcurrentSequence {

        public MoveByEncoderStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double power, int count, boolean stop)
        {
            if (fr != null)
                this.add(new EncoderMotorStep(fr, power, count, stop));
            if (br != null)
                this.add(new EncoderMotorStep(br, power, count, stop));
            if (fl != null)
                this.add(new EncoderMotorStep(fl, power, count, stop));
            if (bl != null)
                this.add(new EncoderMotorStep(bl, power, count, stop));
        }

        public MoveByEncoderStep(DcMotor motors[], double power, int count, boolean stop)
        {
            for (DcMotor em : motors)
                if (em != null)
                    this.add(new EncoderMotorStep(em, power, count, stop));
        }

    }


    // a Sequence that turns an up-to-four-motor robot by applying the given right and left powers for given right and left encoder counts
    static public class TurnByEncoderStep extends ConcurrentSequence {

        public TurnByEncoderStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double rightPower, double leftPower, int rightCount, int leftCount, boolean stop)
        {
            if (fr != null)
                this.add(new EncoderMotorStep(fr, rightPower, rightCount, stop));
            if (br != null)
                this.add(new EncoderMotorStep(br, rightPower, rightCount, stop));
            if (fl != null)
                this.add(new EncoderMotorStep(fl, leftPower, leftCount, stop));
            if (bl != null)
                this.add(new EncoderMotorStep(bl, leftPower, leftCount, stop));
        }

    }


    // some utilities to support "Squirrely Wheels" that move the robot sideways
    // when front and back wheels turn in opposite directions

    // example of a class used to return multiple values from a function call
    public static class MotorPowers {
        public double mFront;
        public double mBack;
        public MotorPowers(double front, double back) {
            mFront = front;
            mBack = back;
        }
        public double Front() { return mFront; }
        public double Back() { return mBack; }
    }

    // this function computes the relative front/back power settings needed to move along a given
    // heading, relative to the current orientation of the robot.
    public static MotorPowers GetSquirrelyWheelMotorPowers(
            double heading    // in degrees, zero = straight ahead, positive CCW, range +-180
    )
    {
        // wrap heading around to acceptable range
        heading = SensorLib.Utils.wrapAngle(heading);

        // compute front and back wheel relative speeds needed to go in desired direction
        double front = 0.0f;
        double back = 0.0f;
        if (heading < 0) {
            if (heading > -90) {
                front = 1.0;
                back = lerp(heading, 0, -90, 1, -1);
            } else {
                front = lerp(heading, -90, -180, 1, -1);
                back = -1.0;
            }
        }
        else {
            if (heading < 90) {
                front = lerp(heading, 0, 90, 1, -1);
                back = 1.0;
            }
            else {
                front = -1.0;
                back = lerp(heading, 90, 180, 1, -1);
            }
        }

        // return results
        return new MotorPowers(front, back);
    }

    // a Step that drives 4 "squirrely wheels" to move the robot in a given direction
    // relative to where it's facing, for a given time.
    static public class MoveSquirrelyByTimeStep extends ConcurrentSequence {

        public MoveSquirrelyByTimeStep(
                DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl,     // motors
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            _MoveSquirrelyByTimeStep(fr, br, fl, bl, heading, power, seconds, stop);
        }

        public MoveSquirrelyByTimeStep(
                DcMotor motors[],  // motors -- assumed order is fr, br, fl, bl
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            _MoveSquirrelyByTimeStep(motors[0], motors[1], motors[2], motors[3], heading, power, seconds, stop);
        }


        // internal function that actually does the constructor stuff for the two different ctors
        protected void _MoveSquirrelyByTimeStep(
                DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl,     // motors
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            // compute relative front and back motor powers needed to move on the desired heading
            MotorPowers mp = GetSquirrelyWheelMotorPowers(heading);

            // create TimedMotorSteps to control the 4 motors
            if (fr != null)
                this.add(new TimedMotorStep(fr, mp.Front()*power, seconds, stop));
            if (br != null)
                this.add(new TimedMotorStep(br, mp.Back()*power, seconds, stop));
            if (fl != null)
                this.add(new TimedMotorStep(fl, mp.Front()*power, seconds, stop));
            if (bl != null)
                this.add(new TimedMotorStep(bl, mp.Back()*power, seconds, stop));
        }
    }


    // some Steps that use Vuforia camera-based input

    // a Step that provides Vuforia-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps assuming order fr, br, fl, bl driving Squirrely Wheels
    // (i.e. wheels that can move the robot in any direction without yawing the robot itself).
    // this step tries to keep the robot on course to a given location on the field.
    static public class VuforiaSquirrelyGuideStep extends Step {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        private ArrayList<SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...
        private VectorF mTargetPosition;                    // target position on field
        private LocationSensor mLocSensor;                  // sensor to use for field location information (e.g. Vuforia)
        private HeadingSensor mYawSensor;                   // sensor to use for robot orientation on field (may be same as LocationSensor)
        private float mError;                               // how close do we have to be to declare "done"
        private boolean mStop;                              // stop motors when target position is reached

        public VuforiaSquirrelyGuideStep(OpMode mode, VectorF targetPosition, LocationSensor locSensor, HeadingSensor yawSensor,
                                         ArrayList<SetPower> motorSteps, float power, float error, boolean stop)
        {
            mOpMode = mode;
            mTargetPosition = targetPosition;
            mLocSensor = locSensor;
            mYawSensor = yawSensor;
            mMotorSteps = motorSteps;
            mPower = power;
            mError = error;
            mStop = stop;
        }

        private void doSearch()
        {
            // turn slowly CCW searching for a target
            final double searchPower = 0.2;
            mMotorSteps.get(0).setPower(searchPower);
            mMotorSteps.get(1).setPower(searchPower);
            mMotorSteps.get(2).setPower(-searchPower);
            mMotorSteps.get(3).setPower(-searchPower);
        }

        public boolean loop()
        {
            super.loop();

            // check for valid location data - if not, do simple search pattern to try to acquire data
            if (!mLocSensor.haveLocation()) {
                doSearch();
                return false;       // not done
            }

            // compute absolute direction vector to target position on field
            VectorF position = mLocSensor.getLocation();
            VectorF dirToTarget = mTargetPosition.subtracted(position);
            dirToTarget.put(2, 0.0f);        // ignore Z when computing distance to target

            // compute absolute field heading to target: zero aligned with Y axis, positive CCW, degrees 0/359
            double headingToTarget = Math.atan2(-dirToTarget.get(0), dirToTarget.get(1));
            headingToTarget *= 180.0/Math.PI;       // to degrees

            // check for valid heading data - if not, do simple search pattern to try to acquire data
            if (!mYawSensor.haveHeading()) {
                doSearch();
                return false;
            }

            // get current orientation of the robot on the field
            double robotYaw = mYawSensor.getHeading();

            // compute relative heading robot should move along
            double robotHeading = headingToTarget - robotYaw;

            // compute motor powers needed to go in that direction
            MotorPowers mp = GetSquirrelyWheelMotorPowers(robotHeading);
            double frontPower = mp.Front() * mPower;
            double backPower = mp.Back() * mPower;

            // reduce motor powers when we're very close to the target position
            final double slowDist = 3.0*mError;   // start slowing down when we're this close
            double distToTarget = dirToTarget.magnitude();
            if (distToTarget < slowDist) {
                frontPower *= distToTarget/slowDist;
                backPower  *= distToTarget/slowDist;
            }

            // are we there yet?
            boolean bDone = distToTarget < mError;     // within an inch of target position?

            // optionally stop motors when we reach the target position
            if (bDone && mStop)
                frontPower = backPower = 0;

            // output debug telemetry
            mOpMode.telemetry.addData("VSGS:", "target position: %s", mTargetPosition.multiplied(1.0f/25.4f).toString());    // inches
            mOpMode.telemetry.addData("VSGS:", "abs heading: %4.1f  distance: %4.1f", headingToTarget, distToTarget/25.4);   // degrees, inches

            // update motors
            // assumed order is fr, br, fl, bl
            mMotorSteps.get(0).setPower(frontPower);
            mMotorSteps.get(1).setPower(backPower);
            mMotorSteps.get(2).setPower(frontPower);
            mMotorSteps.get(3).setPower(backPower);

            return bDone;
        }
    }

    // a Step that uses Vuforia input to drive a SquirrelyWheel robot to a given position on the field.
    // uses a VuforiaSquirrelyGuideStep to adjust power to the 4 motors, assuming order fr, br, fl, bl.
    static public class VuforiaSquirrelyDriveStep extends ConcurrentSequence {

        public VuforiaSquirrelyDriveStep(OpMode mode, VectorF targetPosition, LocationSensor locSensor, HeadingSensor yawSensor,
                                         DcMotor motors[], float power, float error, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);  // always set requested power and return "done"
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on Vuforia input -
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new VuforiaSquirrelyGuideStep(mode, targetPosition, locSensor, yawSensor, steps, power, error, stop));

        }

        // the base class loop function does all we need --
        // since the motors always return done, the composite step will return "done" when
        // the GuideStep says it's done, i.e. we've reached the target location.

    }

    // a Step that provides Vuforia-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps assuming order fr, br, fl, bl driving normal wheels
    // this step tries to keep the robot on course to a given location on the field.
    static public class VuforiaGuideStep extends Step {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private OpMode mOpMode;                             // needed so we can log output (may be null)
        private ArrayList<SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...
        private VectorF mTargetPosition;                    // target position on field
        private LocationSensor mLocSensor;                  // sensor to use for field location information (e.g. Vuforia)
        private HeadingSensor mYawSensor;                   // sensor to use for robot orientation on field (may be same as LocationSensor)
        private float mError;                               // how close do we have to be to declare "done"
        private SensorLib.PID mPid;                         // PID controller to compute motor corrections from Vuforia heading data
        private double mPrevTime;                           // time of previous loop() call
        private boolean mStop;                              // stop motors when target position is reached

        public VuforiaGuideStep(OpMode mode, VectorF targetPosition, LocationSensor locSensor, HeadingSensor yawSensor,
                                ArrayList<SetPower> motorSteps, float power, float error, boolean stop)
        {
            mOpMode = mode;
            mTargetPosition = targetPosition;
            mLocSensor = locSensor;
            mYawSensor = yawSensor;
            mMotorSteps = motorSteps;
            mPower = power;
            mError = error;
            mStop = stop;

            // parameters of the PID controller for this sequence
            float Kp = 0.035f;        // motor power proportional term correction per degree of deviation
            float Ki = 0.02f;         // ... integrator term
            float Kd = 0;             // ... derivative term
            float KiCutoff = 3.0f;    // maximum angle error for which we update integrator

            // create a PID controller for the sequence
            mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm
        }

        public boolean loop()
        {
            super.loop();

            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by OpMode
            }

            float rightPower = 0;
            float leftPower = 0;
            boolean bDone = false;

            // check for valid location and heading input --
            // if either is absent, do a slow circling maneuver to try to acquire
            if (!mLocSensor.haveLocation() || !mYawSensor.haveHeading()) {
                rightPower = 0.2f;
                leftPower  = -rightPower;
            }

            else {
                // compute absolute direction vector to target position on field
                VectorF position = mLocSensor.getLocation();
                if (position == null)
                    return false;       // we have an inconsistency problem but we're not done

                VectorF dirToTarget = mTargetPosition.subtracted(position);
                dirToTarget.put(2, 0.0f);        // ignore Z when computing distance to target

                // compute absolute field heading to target: zero aligned with Y axis, positive CCW, degrees 0/359
                float headingToTarget = (float) (Math.atan2(-dirToTarget.get(0), dirToTarget.get(1)));
                headingToTarget *= 180.0 / Math.PI;       // to degrees

                // try to keep the robot facing toward the corner between the sides of the fields with Vuforia targets --
                // if direction to target is away from that corner of the field, run backwards to the target; otherwise forwards.
                // i.e. if directionToTarget dot (-1,-1) < 0, then we're heading away from the "good" corner
                boolean destAwayFromTargets = dirToTarget.dotProduct(new VectorF(-1, -1, 0)) < 0;

                float robotHeading = mYawSensor.getHeading();     // get latest reading from direction sensor
                // convention is positive angles CCW, wrapping from 359-0

                // if the target position is behind us, reverse the robot orientation direction so
                // we can compute correction to go in the desired direction toward the target (backwards)
                if (destAwayFromTargets) {
                    robotHeading = SensorLib.Utils.wrapAngle(robotHeading + 180);
                }

                float error = SensorLib.Utils.wrapAngle(robotHeading - headingToTarget);   // deviation from desired heading
                // deviations to left are positive, to right are negative

                // compute delta time since last call -- used for integration time of PID step
                double time = mOpMode.getRuntime();
                double dt = time - mPrevTime;
                mPrevTime = time;

                // feed error through PID to get motor power correction value
                float correction = -mPid.loop(error, (float) dt);

                // compute new right/left motor powers
                float powerDir = (destAwayFromTargets) ? -mPower : mPower;
                rightPower = Range.clip(powerDir + correction, -1, 1);
                leftPower = Range.clip(powerDir - correction, -1, 1);

                // reduce motor powers when we're very close to the target position
                final double slowDist = 3.0*mError;   // start slowing down when we're this close
                double distToTarget = dirToTarget.magnitude();
                if (distToTarget < slowDist) {
                    leftPower *= distToTarget / slowDist;
                    rightPower *= distToTarget / slowDist;
                }

                // are we there yet?
                bDone = distToTarget < mError;     // within given distance of target position?

                // optionally stop motors when we reach the target position
                if (bDone && mStop)
                    leftPower = rightPower = 0;

                // output debug telemetry
                mOpMode.telemetry.addData("VGS:", "abs heading: %4.1f  distance: %4.1f", headingToTarget, distToTarget/25.4);   // degrees, inches
            }

            // set the motor powers -- handle both time-based and encoder-based motor Steps
            // assumed order is right motors followed by an equal number of left motors
            int i = 0;
            for (SetPower ms : mMotorSteps) {
                ms.setPower((i++ < mMotorSteps.size()/2) ? rightPower : leftPower);
            }

            // output debug telemetry
            mOpMode.telemetry.addData("VGS:", "target position: %s", mTargetPosition.multiplied(1.0f/25.4f).toString());    // inches

            return bDone;
        }
    }

    // a Step that uses Vuforia input to drive a normal wheel robot to a given position on the field.
    // uses a VuforiaGuideStep to adjust power to the 4 motors, assuming order fr, br, fl, bl.
    static public class VuforiaDriveStep extends ConcurrentSequence {

        public VuforiaDriveStep(OpMode mode, VectorF targetPosition, LocationSensor locSensor, HeadingSensor yawSensor,
                                DcMotor motors[], float power, float error, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);  // always set requested power and return "done"
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on Vuforia input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new VuforiaGuideStep(mode, targetPosition, locSensor, yawSensor, steps, power, error, stop));

        }

        // the base class loop function does all we need --
        // since the motors always return done, the composite step will return "done" when
        // the GuideStep says it's done, i.e. we've reached the target location.

    }


    // timer
    static public class Timer {
        long mStartTime;
        double mSeconds;

        public Timer(double seconds) {
            mStartTime = 0L;        // creation time is NOT start time
            mSeconds = seconds;
        }

        public void start() {
            mStartTime = System.nanoTime();
        }

        // return elapsed time in seconds since timer was created or restarted
        public double elapsed() {
            return (double) (System.nanoTime() - mStartTime) / (double) TimeUnit.SECONDS.toNanos(1L);
        }

        public double remaining() {
            return mSeconds - elapsed();
        }

        public boolean done() {
            return (remaining() <= 0);
        }
    }


    // test hardware classes -- useful for testing when no hardware is available.
    // these are primarily intended for use in testing autonomous mode code, but could
    // also be useful for testing tele-operator modes.

    static public class TestHardware implements HardwareDevice {
        public HardwareDevice.Manufacturer getManufacturer() { return Manufacturer.Unknown; }
        public String getDeviceName() { return "AutoLib_TestHardware"; }
        public String getConnectionInfo() { return "connection info unknown"; }
        public int getVersion() { return 0; }
        public void resetDeviceConfigurationForOpMode() {}
        public void close() {}
    }

    // a dummy DcMotor that just logs commands we send to it --
    // useful for testing Motor code when you don't have real hardware handy
    static public class TestMotor extends TestHardware implements DcMotor {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this motor
        double mPower;      // current power setting
        DcMotor.RunMode mMode;
        int mTargetPosition;
        int mCurrentPosition;
        boolean mPowerFloat;
        Direction mDirection;
        ZeroPowerBehavior mZeroPowerBehavior;
        int mMaxSpeed;
        MotorConfigurationType mMotorType;

        public TestMotor(String name, OpMode opMode) {
            super();     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPower = 0.0;
            mMaxSpeed = 0;
            mMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            mTargetPosition = 0;
            mCurrentPosition = 0;
            mPowerFloat = false;
            mDirection = Direction.FORWARD;
            mZeroPowerBehavior = ZeroPowerBehavior.FLOAT;
        }

        @Override       // override all the functions of the real DcMotor class that touch hardware
        public void setPower(double power) {
            mPower = power;
            mOpMode.telemetry.addData(mName, " power: " + String.valueOf(mPower));
        }

        public double getPower() {
            return mPower;
        }

        public void close() {
            mOpMode.telemetry.addData(mName, " close();");
        }

        public boolean isBusy() {
            return false;
        }

        public void setPowerFloat() {
            mPowerFloat = true;
            mOpMode.telemetry.addData(mName, " setPowerFloat();");
        }

        public boolean getPowerFloat() {
            return mPowerFloat;
        }

        public void setMaxSpeed(int encoderTicksPerSecond)
        {
            mMaxSpeed = encoderTicksPerSecond;
            mOpMode.telemetry.addData(mName, "maxSpeed: " + String.valueOf(encoderTicksPerSecond));
        }

        public int getMaxSpeed() { return mMaxSpeed; }

        public void setTargetPosition(int position) {
            mTargetPosition = position;
            mOpMode.telemetry.addData(mName, "target: " + String.valueOf(position));
        }
        public int getTargetPosition() {
            return mTargetPosition;
        }

        public int getCurrentPosition() {
            return mTargetPosition;
        }

        public void setMode(DcMotor.RunMode mode) {
            this.mMode = mode;
            mOpMode.telemetry.addData(mName, "run mode: " + String.valueOf(mode));
        }

        public DcMotor.RunMode getMode() {
            return this.mMode;
        }

        public void setDirection(Direction direction)
        {
            mDirection = direction;
            mOpMode.telemetry.addData(mName, "direction: " + String.valueOf(direction));
        }

        public Direction getDirection() { return mDirection; }

        public String getConnectionInfo() {
            return mName + " port: unknown";
        }

        public DcMotorController getController()
        {
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            mOpMode.telemetry.addData(mName, "resetDeviceConfigurationForOpMode: ");
        }

        public int getPortNumber()
        {
            return 0;
        }

        public ZeroPowerBehavior getZeroPowerBehavior()
        {
            return mZeroPowerBehavior;
        }

        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior)
        {
            mZeroPowerBehavior = zeroPowerBehavior;
            mOpMode.telemetry.addData(mName, "zeroPowerBehavior: " + String.valueOf(zeroPowerBehavior));
        }

        public String getDeviceName() { return "AutoLib_TestMotor: " + mName; }

        public void setMotorType(MotorConfigurationType motorType) { mMotorType = motorType; }

        public MotorConfigurationType getMotorType() { return mMotorType; }

    }

    // a dummy Servo that just logs commands we send to it --
    // useful for testing Servo code when you don't have real hardware handy
    static public class TestServo extends TestHardware implements Servo {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this servo
        double mPosition;   // current target position
        Direction mDirection;
        double mScaleMin;
        double mScaleMax;

        public TestServo(String name, OpMode opMode) {
            super();     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPosition = 0.0;
        }

        @Override       // this function overrides the setPower() function of the real DcMotor class
        public void setPosition(double position) {
            mPosition = position;
            mOpMode.telemetry.addData(mName, " position: " + String.valueOf(mPosition));
            mDirection = Direction.FORWARD;
        }

        @Override       // this function overrides the getPower() function of the real DcMotor class
        public double getPosition() {
            return mPosition;
        }

        @Override       // override all other functions of Servo that touch the hardware
        public String getConnectionInfo() {
            return mName + " port: unknown";
        }

        public HardwareDevice.Manufacturer getManufacturer() { return Manufacturer.Unknown; }

        public void resetDeviceConfigurationForOpMode() {
            mOpMode.telemetry.addData(mName, "resetDeviceConfigurationForOpMode: ");
        }

        public Servo.Direction getDirection() { return mDirection; }
        public void setDirection(Servo.Direction direction) {
            mDirection = direction;
            mOpMode.telemetry.addData(mName, "direction: " + String.valueOf(mDirection));
        }

        public void scaleRange(double min, double max)
        {
            mScaleMin = min;
            mScaleMax = max;
        }

        public ServoController getController() { return null; }
        public String getDeviceName() { return "AutoLib_TestServo: " + mName; }
        public int getPortNumber()
        {
            return 0;
        }
        public int getVersion() { return 0; }

        public void close() {}

    }

    // a dummy Gyro that just returns default info --
    // useful for testing Gyro code when you don't have real hardware handy
    static public class TestGyro extends TestHardware implements GyroSensor {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this gyro

        public TestGyro(String name, OpMode opMode) {
            super();
            mOpMode = opMode;
            mName = name;
        }

        public void calibrate() {}
        public boolean isCalibrating() { return false; }
        public int getHeading() { return 0; }
        public double getRotationFraction() { return 0; }
        public int rawX() { return 0; }
        public int rawY() { return 0; }
        public int rawZ() { return 0; }
        public void resetZAxisIntegrator() {}
        public String status() { return "Status okay"; }
        public String getDeviceName() { return "AutoLib_TestGyro: " + mName; }

    }

    // a dummy ColorSensor that just returns default info --
    // useful for testing ColorSensor code when you don't have real hardware handy
    static public class TestColorSensor extends TestHardware implements ColorSensor {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this gyro

        public TestColorSensor(String name, OpMode opMode) {
            super();
            mOpMode = opMode;
            mName = name;
        }

        /**
         * Get the Red values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int red() { return 0; }

        /**
         * Get the Green values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int green() { return 0; }

        /**
         * Get the Blue values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int blue() { return 255; }

        /**
         * Get the amount of light detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int alpha() { return 0; }

        /**
         * Get the "hue"
         * @return hue
         */
        //@ColorInt
        public int argb() { return 0; }

        /**
         * Enable the LED light
         * @param enable true to enable; false to disable
         */
        public void enableLed(boolean enable) {}

        /**
         * Set the I2C address to a new value.
         *
         */
        public void setI2cAddress(I2cAddr newAddress) {}

        /**
         * Get the current I2C Address of this object.
         * Not necessarily the same as the I2C address of the actual device.
         *
         * Return the current I2C address.
         * @return current I2C address
         */
        public I2cAddr getI2cAddress() {return I2cAddr.zero();}

        public String status() { return "Status okay"; }
        public String getDeviceName() { return "AutoLib_TestGyro: " + mName; }

    }

    // define interface to Factory that creates various kinds of hardware objects
    static public interface HardwareFactory {
        public DcMotor getDcMotor(String name);
        public Servo getServo(String name);
        public GyroSensor getGyro(String name);
        public ColorSensor getColorSensor(String name);
    }

    // this implementation generates test-hardware objects that just log data
    static public class TestHardwareFactory implements HardwareFactory {
        OpMode mOpMode;     // needed for logging data

        public TestHardwareFactory(OpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            return new TestMotor(name, mOpMode);
        }

        public Servo getServo(String name){
            return new TestServo(name, mOpMode);
        }

        public GyroSensor getGyro(String name){
            return new TestGyro(name, mOpMode);
        }

        public ColorSensor getColorSensor(String name){
            return new TestColorSensor(name, mOpMode);
        }
    }

    // this implementation gets real hardware objects from the hardwareMap of the given OpMode
    static public class RealHardwareFactory implements HardwareFactory {
        OpMode mOpMode;     // needed for access to hardwareMap

        public RealHardwareFactory(OpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            DcMotor motor = null;
            try {
                motor = mOpMode.hardwareMap.dcMotor.get(name);
            }
            catch (Exception e) {
                // okay -- just return null (absent) for this motor
            }

            // just to make sure - a previous OpMode may have set it differently ...
            if (motor != null)
                motor.setDirection(DcMotor.Direction.FORWARD);

            return motor;
        }

        public Servo getServo(String name){
            Servo servo = null;
            try {
                servo = mOpMode.hardwareMap.servo.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }

            // just to make sure - a previous OpMode may have set it differently ...
            if (servo != null)
                servo.setDirection(Servo.Direction.FORWARD);

            return servo;
        }

        public GyroSensor getGyro(String name){
            GyroSensor gyro = null;
            try {
                gyro = mOpMode.hardwareMap.gyroSensor.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }
            return gyro;
        }

        public ColorSensor getColorSensor(String name){
            ColorSensor cs = null;
            try {
                cs = mOpMode.hardwareMap.colorSensor.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }
            return cs;
        }

    }

}



