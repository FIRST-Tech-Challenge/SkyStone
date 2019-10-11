package teamcode.common;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TTArm {
    private final double INCHES_TO_TICKS = 2912.0/9.42;
    private final double TICKS_TO_INCHES = 9.42/2912.0;
    private final DcMotor armLift;
    private final Servo armWrist, armClaw;
    private static final double TICK_ERROR = 25.0;
    private double lastPosition = 0;

    public TTArm(HardwareMap hardwareMap) {
        armLift = hardwareMap.get(DcMotor.class, TTHardwareComponentNames.ARM_LIFT);
        armWrist = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_WRIST);
        armClaw = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_CLAW);
    }

    public void armMove(double inches) {
        int ticks = (int) (inches * INCHES_TO_TICKS);
        armLift.setTargetPosition(ticks);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(1.0);
        while(!nearTarget()) {
            brake();
        }
    }

    private boolean nearTarget() {
            int targetPosition = armLift.getTargetPosition();
            int currentPosition = armLift.getCurrentPosition();
            double ticksFromTarget = Math.abs(targetPosition - currentPosition);
            if (ticksFromTarget > TICK_ERROR) {
                return false;
            }
        return true;
    }

    public void brake() {
        armLift.setPower(0.0);
    }

    public void armLift(){
        double startingPosition = lastPosition * TICKS_TO_INCHES;
        armMove(startingPosition + 5.0);
        lastPosition = armLift.getCurrentPosition();
    }
    public void armLower(){
        armMove(0);
    }
    public void rotateClaw(double position) {
        armWrist.setPosition(position);
    }

    public void rotateWrist(double position){
        armClaw.setPosition(position);
    }

    public double getWristPos(){
        return this.armWrist.getPosition();
    }

    public double getClawPos(){
        return this.armClaw.getPosition();
    }
}
