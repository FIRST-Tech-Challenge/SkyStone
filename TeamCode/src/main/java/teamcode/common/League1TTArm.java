package teamcode.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.TimerTask;

public class League1TTArm {

    private static final double LIFT_POWER = 0.5;
    /**
     * Inches per seconds at max power.
     */
    private static final double LIFT_INCHES_PER_SECONDS = 10.0;

    private final CRServo lift;

    public League1TTArm(HardwareMap hardwareMap) {
        lift = hardwareMap.get(CRServo.class, TTHardwareComponentNames.ARM_LIFT);
    }

    public void lift(double inches) {
        lift.setPower(LIFT_POWER);
        long durationMilis = (long) (inches / LIFT_INCHES_PER_SECONDS * 1000);
        TimerTask stop = new TimerTask() {
            @Override
            public void run() {
                lift.setPower(0.0);
            }
        };
        TTOpMode.currentOpMode().getTimer().schedule(stop, durationMilis);
        while (lift.getPower() != 0.0) ;
    }

}
