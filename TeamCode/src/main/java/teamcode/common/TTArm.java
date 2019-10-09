package teamcode.common;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TTArm {
    private final DcMotor armLift;
    private final Servo armWrist, armClaw;

    public TTArm(HardwareMap hardwareMap) {
        armLift = hardwareMap.get(DcMotor.class, TTHardwareComponentNames.ARM_LIFT);
        armWrist = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_WRIST);
        armClaw = hardwareMap.get(Servo.class, TTHardwareComponentNames.ARM_CLAW);
    }

    public void armLift(float power) {
        armLift.setPower(power);
    }

    public void armLower(float power) {
         armLift.setPower(power);
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
