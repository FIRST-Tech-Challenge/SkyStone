package teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class TTOpMode extends LinearOpMode {

    private static TTOpMode opMode;

    @Override
    public final void runOpMode() {
        opMode = this;
        TTTimer.init();
        onInitialize();
        waitForStart();
        onStart();
        while (opModeIsActive()) ; // this is necessary if code is being run on separate threads
        onStop();
        TTTimer.cancel();
    }

    public static TTOpMode getOpMode() {
        return opMode;
    }

    protected abstract void onInitialize();

    protected abstract void onStart();

    protected abstract void onStop();

}
