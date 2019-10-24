package teamcode.common;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class TTArm {

    private final double INCHES_TO_TICKS = 2912.0/9.42;
    private final double TICKS_TO_INCHES = 9.42/2912.0;
    private static final double TICK_ERROR = 25;
    private final CRServo armLift;
    private final Servo armClaw;

    private double lastPosition = 0;


    public TTArm(HardwareMap hardwareMap) {
        armLift = hardwareMap.get(CRServo.class, TTHardwareComponentNames.ARM_LIFT);
        armClaw = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_CLAW);
    }

    public void brake() {
        armLift.setPower(0.0);
    }
    public void armMove(double power) {
        armLift.setPower(power);
    }
    public void rotateClaw(double position) {
        armClaw.setPosition(position);
    }

    public double getClawPos(){
        return this.armClaw.getPosition();
    }
}
