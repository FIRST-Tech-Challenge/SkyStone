package teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Timer;

public abstract class TTOpMode extends LinearOpMode {

    private static TTOpMode opMode;

    private Timer timer;

    public static TTOpMode currentOpMode() {
        return opMode;
    }

    @Override
    public final void runOpMode() {
        opMode = this;
        timer = new Timer();
        onInitialize();
        waitForStart();
        onStart();
        onStop();
        timer.cancel();
    }

    protected abstract void onInitialize();

    protected abstract void onStart();

    protected abstract void onStop();

    public Timer getTimer() {
        return timer;
    }

}
