package teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class TTOpMode extends LinearOpMode {

    private static TTOpMode opMode;
    private TTHardwareManager.TTHardwareRestriction[] hardwareRestrictions;
    private TTRobot robot;

    @Override
    public void runOpMode() {
        opMode = this;
        TTTimer.init();
        onInitialize();
        robot = new TTRobot(hardwareMap, hardwareRestrictions);
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

    protected void setHardwareRestrictions(TTHardwareManager.TTHardwareRestriction... restrictions) {
        hardwareRestrictions = restrictions;
    }

    protected abstract void onInitialize();

    protected abstract void onStart();

}
