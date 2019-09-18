package org.firstinspires.ftc.teamcode._Test._Motors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;

/**
 * Created by phanau on 12/7/18.
 * Test Servo step using DoUntilStep to loop the test until you hit Stop
 */

class UntilStep extends AutoLib.Step {
    @Override
    public boolean loop() {
        return false;       // for now, just loop forever (i.e. Until is always false)
    }
}

@Autonomous(name="Test: TestServoSteps", group ="Test")
//@Disabled
public class TestServoSteps extends OpMode {

    Servo mMotor;
    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done

    public TestServoSteps() {
    }

    public void init() {
        // get hardware
        AutoLib.HardwareFactory mf = null;
        final boolean debug = false;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the servo: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        mMotor = mf.getServo("left_hand");      // for compatability with ConceptScanServo

        // create the root sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // create the DoUntil loop Sequences
        AutoLib.Sequence mDo = new AutoLib.LinearSequence();
        AutoLib.Step mUntil = new UntilStep();
        mSequence.add(new AutoLib.DoUntilStep(mDo, mUntil));

        // add a Step sequence that rotates a servo to various positions
        // see if ServoStep successfully waits long enough to reach the target position
        // given "fullRangeTime" for this servo
        double fullRangeTime = 2.0;         // Servo takes (no more than) this time to go from 0..1
        mDo.add(new AutoLib.ServoStep(mMotor, 0.0, fullRangeTime));
        mDo.add(new AutoLib.ServoStep(mMotor, 1.0, fullRangeTime));
        mDo.add(new AutoLib.ServoStep(mMotor, 0.0, fullRangeTime));
        mDo.add(new AutoLib.ServoStep(mMotor, 0.5, fullRangeTime));
        mDo.add(new AutoLib.ServoStep(mMotor, 0.0, fullRangeTime));
        int n = 8;
        for (int i=0; i<=n; i++)
            mDo.add(new AutoLib.ServoStep(mMotor, i*(1.0/n), fullRangeTime));
        for (int i=n; i>=0; i--)
            mDo.add(new AutoLib.ServoStep(mMotor, i*(1.0/n), fullRangeTime));

        // initially, sequence is not done
        bDone = false;
    }

    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    public void stop() {
    }

}
