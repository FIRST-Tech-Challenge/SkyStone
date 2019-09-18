package org.firstinspires.ftc.teamcode._Test._Drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;


/**
 * simple example of using a Step that uses gyro input to drive along a given course for a given time
 * Created by phanau on 1/3/16 as SensorDriveTestOp, modified 1/22/16 to use Gyro, renamed GyroDriveTestOp 2/16/16.
 * Revised 23 Nov 2018 to use IMU in REV Robotics hub.
 */


// simple example sequence that tests either of gyro-based AzimuthCountedDriveStep or AzimuthTimedDriveStep to drive along a square path
@Autonomous(name="Test: REV IMU Drive Test", group ="Test")
//@Disabled
public class GyroDriveTestOp extends OpMode {

    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    BNO055IMUHeadingSensor mGyro;           // gyro to use for heading information
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;                     // PID controller for the sequence

    // parameters of the PID controller for this sequence - assumes 20-gear motors (fast)
    float Kp = 0.02f;        // motor power proportional term correction per degree of deviation
    float Ki = 0.025f;         // ... integrator term
    float Kd = 0;             // ... derivative term
    float KiCutoff = 10.0f;    // maximum angle error for which we update integrator

    @Override
    public void init() {
        bSetup = false;      // start out in Kp/Ki setup mode
        AutoLib.HardwareFactory mf = null;
        final boolean debug = false;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        mMotors = new DcMotor[4];
        mMotors[0] = mf.getDcMotor("fr");
        mMotors[1] = mf.getDcMotor("br");
        (mMotors[2] = mf.getDcMotor("fl")).setDirection(DcMotor.Direction.REVERSE);
        (mMotors[3] = mf.getDcMotor("bl")).setDirection(DcMotor.Direction.REVERSE);

        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mGyro = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mGyro.init(7);  // orientation of REV hub in my ratbot
        mGyro.setDegreesPerTurn(355.0f);     // appears that's what my IMU does ...

        // create a PID controller for the sequence
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float movePower = 0.4f;
        float turnPower = 0.25f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a bunch of timed "legs" to the sequence - use Gyro heading convention of positive degrees CW from initial heading
        float tol = 5.0f;   // tolerance in degrees
        float timeout = 2.0f;   // seconds
        // test "scanning" left and right e.g. 30 degrees
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 30, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.LogTimeStep(this, "left 30", 2));
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, -30, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.LogTimeStep(this, "right 30", 2));

        // turn 4 quadrants in place
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 0, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.LogTimeStep(this, "course 0", 1));
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 90, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.LogTimeStep(this, "course 90", 1));
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 180, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.LogTimeStep(this, "course 180", 1));
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 270, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.LogTimeStep(this, "course 270", 1));
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 0, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.LogTimeStep(this, "course 0", 1));

        boolean bUseEncoders = false;
        if (bUseEncoders) {
            // add a bunch of encoder-counted "legs" to the sequence - use Gyro heading convention of positive degrees CCW from initial heading
            int leg = 4000;  // motor-encoder count along each leg of the polygon
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 0, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 90, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 180, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, -90, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 0, mGyro, mPid, mMotors, turnPower, tol, timeout));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 90, mGyro, mPid, mMotors, movePower, leg, true));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 0, mGyro, mPid, mMotors, movePower, leg, true));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, -90, mGyro, mPid, mMotors, movePower, leg, true));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 180, mGyro, mPid, mMotors, movePower, leg, true));
        }
        else  {
            // drive a "box" CCW and then again CW - see if we end up where we started ...
            float leg = debug ? 6.0f : 1.5f;  // time along each leg of the polygon
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 0, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 90, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 180, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, -90, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 0, mGyro, mPid, mMotors, turnPower, tol, timeout));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 90, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 0, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, -90, mGyro, mPid, mMotors, movePower, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 180, mGyro, mPid, mMotors, movePower, leg, false));
        }
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 0, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.MoveByTimeStep(mMotors, 0, 0, true));     // stop all motors

        // start out not-done
        bDone = false;
    }

    @Override
    public void loop() {

        if (gamepad1.y)
            bSetup = true;      // Y button: enter "setup mode" using controller inputs to set Kp and Ki
        if (gamepad1.x)
            bSetup = false;     // X button: exit "setup mode"
        if (bSetup) {           // "setup mode"
            // adjust PID parameters by joystick inputs
            Kp -= (gamepad1.left_stick_y * 0.0001f);
            Ki -= (gamepad1.right_stick_y * 0.0001f);
            // update the parameters of the PID used by all Steps in this test
            mPid.setK(Kp, Ki, Kd, KiCutoff);
            // log updated values to the operator's console
            telemetry.addData("Kp = ", Kp);
            telemetry.addData("Ki = ", Ki);
            return;
        }

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
    }
}

