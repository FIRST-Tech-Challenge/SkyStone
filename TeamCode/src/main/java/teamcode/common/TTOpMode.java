package teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Timer;

public abstract class TTOpMode extends LinearOpMode {

    private static TTOpMode opMode;


    private TTRobot robot;

    private Timer timer;

    public static TTOpMode currentOpMode() {
        return opMode;
    }


    @Override
    public final void runOpMode() {
        opMode = this;
        timer = new Timer();
        robot = new TTRobot(hardwareMap);
        onInitialize();
        waitForStart();
        onStart();
        onStop();
        timer.cancel();
        while (opModeIsActive()) ; // this is necessary if code is being run on separate threads
    }

    public static TTOpMode getOpMode() {
        return opMode;

    }

    public TTRobot getRobot() {
        return robot;
    }

    protected abstract void onInitialize();

    protected abstract void onStart();

    protected abstract void onStop();


    public Timer getTimer() {

        return timer;
    }

}
