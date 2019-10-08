package teamcode.common;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class TTArm {
    private final double INCHES_TO_TICKS = 309.13;
    private final double TICKS_TO_INCHES = 0.0032349;
    private final DcMotor armLift;
    private final Servo armWrist, armClaw;
    private static final double TICK_ERROR = 25.0;
    private double lastPosition = 0;

    public TTArm(HardwareMap hardwareMap) {
        armLift = hardwareMap.get(DcMotor.class, HardwareComponentNames.ARM_LIFT);
        armWrist = hardwareMap.get(Servo.class, HardwareComponentNames.ARM_WRIST);
        armClaw = hardwareMap.get(Servo.class, HardwareComponentNames.ARM_CLAW);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        armClaw.setPosition(position);
    }

    public void rotateWrist(double position){
        armWrist.setPosition(position);
    }

    public double getWristPos(){
        return this.armWrist.getPosition();
    }

    public double getClawPos(){
        return this.armClaw.getPosition();
    }

    public int getArmLiftPos() {
        return this.armLift.getCurrentPosition();
    }
    public int getArmTarget(){
        return  this.armLift.getTargetPosition();
    }
}
