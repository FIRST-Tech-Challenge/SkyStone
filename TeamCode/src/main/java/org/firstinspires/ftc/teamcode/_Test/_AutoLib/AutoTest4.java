package org.firstinspires.ftc.teamcode._Test._AutoLib;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.ToggleButton;


/**
 * A test example of autonomous opmode programming using AutoLib classes.
 * This one tests the Switch Step construct.
 * Created by phanau on 1/25/19.
 */

class myDynamicCaseStep extends AutoLib.Step {
    OpMode mOpmode;
    AutoLib.SwitchStep2 mTarget;
    ToggleButton mToggleButton;

    public myDynamicCaseStep(OpMode opmode) {
        mOpmode = opmode;
        mToggleButton = new ToggleButton(false, 3, 0);
    }

    public void setTarget(AutoLib.SwitchStep2 target) {
        mTarget = target;
    }

    public boolean loop() {
        mToggleButton.process(mOpmode.gamepad1.x);
        mTarget.setInteger(mToggleButton.value());
        return true;
    }
}

class mySwitchStep extends AutoLib.SwitchStep {
    int mCase = 0;
    OpMode mOpMode;

    public mySwitchStep(AutoLib.Step[] cases, boolean once, OpMode opmode) {
        super(cases, once);
        mOpMode = opmode;
    }

    int caseTest() {
        mCase = (mCase+1) % 3;  // this should return 1 the first time it's called
        mOpMode.telemetry.addData("mySwitchStep.caseTest()", mCase);
        return mCase;
    }
}

@Autonomous(name="Test: AutoLib Switch Test", group ="Test")
//@Disabled
public class AutoTest4 extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done

    public AutoTest4() {
    }

    public void init() {
        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a switch Step to the root Sequence that runs one of 3 Steps depending on
        // the case continuously returned by the case step - controlled by an X button 3-way toggle
        AutoLib.Step[] steps = new AutoLib.Step[3];
        int count = 2000;
        steps[0] = new AutoLib.LogCountStep(this,"case 0", count);
        steps[1] = new AutoLib.LogCountStep(this,"case 1", count);
        steps[2] = new AutoLib.LogCountStep(this,"case 2", count);
        myDynamicCaseStep caseStep = new myDynamicCaseStep(this);
        AutoLib.SwitchStep2 switchStep = new AutoLib.SwitchStep2(caseStep, steps, false);
        caseStep.setTarget(switchStep);
        mSequence.add(switchStep);

        // add a switch Step that runs the appropriate sequence (step), which should be "case 1".
        mSequence.add(new mySwitchStep(steps, true, this));

        // start out not-done
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
        telemetry.addData("stop() called", "");
    }
}
