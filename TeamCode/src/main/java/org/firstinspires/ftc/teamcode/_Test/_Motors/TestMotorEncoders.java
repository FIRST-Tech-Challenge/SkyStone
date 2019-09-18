package org.firstinspires.ftc.teamcode._Test._Motors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;

/**
 * Created by phanau on 2/23/18.

 * Test hardware gyro
 */
@Autonomous(name="Test: Motor Encoder Test 1", group ="Test")
//@Disabled
public class TestMotorEncoders extends OpMode {

    DcMotor mMotor1, mMotor2;
    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done

    public TestMotorEncoders() {
    }

    public void init() {
        // get hardware
        AutoLib.HardwareFactory mf = null;
        final boolean debug = false;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        (mMotor1 = mf.getDcMotor("fl")).setDirection(DcMotor.Direction.REVERSE);
        mMotor2 = mf.getDcMotor("br");

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a Step sequence that rotates a motor N revs forward and then backward to starting position
        double power = 0.4;
        int countPerTurn = 28*20;    // shaft encoder at 28 ppr * 20:1 gearbox
        boolean stop = true;
        int turns = 10;
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor1, power, turns*countPerTurn, stop));      // forward
        mSequence.add(new AutoLib.LogTimeStep(this, "wait", 1.0));
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor1, power, -turns*countPerTurn, stop));     // return to initial position
        mSequence.add(new AutoLib.LogTimeStep(this, "wait", 1.0));
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor2, power, turns*countPerTurn, stop));      // forward
        mSequence.add(new AutoLib.LogTimeStep(this, "wait", 1.0));
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor2, power, -turns*countPerTurn, stop));     // return to initial position

        // test holding position at full power within a sequence (simulating rover hanging from lander).
        // the motors all release when stop is hit, but here we explicitly release them after a given time just to test that action.
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor1, 1.0, 0, false));     // hold final position
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor2, 1.0, 0, false));     // hold final position
        mSequence.add(new AutoLib.LogTimeStep(this, "hang", 10.0));          // ... for this time
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor1, 0.0, 0, true));      // release motor
        mSequence.add(new AutoLib.EncoderMotorStep(mMotor2, 0.0, 0, true));      // release motor

        // test using position holding step to lock motor =immediately= from now (init) until start
        new AutoLib.EncoderMotorStep(mMotor1, 1.0, 0, false).loop();        // create step and run it once
        new AutoLib.EncoderMotorStep(mMotor2, 1.0, 0, false).loop();        // create step and run it once
    }

    public void loop() {
        // post some feedback from the motors to the operator console
        telemetry.addData("count 1", mMotor1.getCurrentPosition());
        telemetry.addData("count 2", mMotor2.getCurrentPosition());

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    public void stop() {
    }

}
