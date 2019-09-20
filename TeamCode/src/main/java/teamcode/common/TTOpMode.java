package teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class TTOpMode extends LinearOpMode {

    private static TTOpMode opMode;

    private TTRobot robot;
    private TTHardwareManager.TTHardwareRestriction hardwareRestriction;

    @Override
    public void runOpMode() {
        opMode = this;
        TTTimer.init();
        onInitialize();
        if (hardwareRestriction == null) {
            hardwareRestriction = TTHardwareManager.TTHardwareRestriction.NONE;
        }
        robot = new TTRobot(hardwareMap, hardwareRestriction);
        waitForStart();
        onStart();
        while (opModeIsActive()) ; // this is necessary if code is being run on separate threads
        TTTimer.cancel();
    }

    public static TTOpMode getOpMode() {
        return opMode;
    }

    public TTRobot getRobot() {
        return robot;
    }

    protected abstract void onInitialize();

    protected abstract void onStart();

    /**
     * Invoke this method in {@link TTOpMode#onInitialize} implementations to set a hardware
     * restriction for testing purposes.
     */
    protected void setHardwareRestriction(TTHardwareManager.TTHardwareRestriction restriction) {
        this.hardwareRestriction = restriction;
    }

}
