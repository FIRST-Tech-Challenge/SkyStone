package org.firstinspires.ftc.teamcode.Skystone.Auto.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionState;

public class MotionAction {
    private DcMotor actionMotor;
    private Servo actionServo;

    private double servoPosition; // servo
    private double motorPower; // motor

    private double delayStartTime;

    private boolean isLocationToggle;

    private ActionState status = ActionState.PENDING;

    public MotionAction(DcMotor actionMotor, double motorPower, double delayStartTime) {
        this.actionMotor = actionMotor;
        this.motorPower = motorPower;
        this.delayStartTime = delayStartTime;
    }

    public MotionAction(Servo actionServo, double servoPosition, double delayStartTime) {
        this.actionServo = actionServo;
        this.servoPosition = servoPosition;
        this.delayStartTime = delayStartTime;
    }

    public MotionAction(Servo actionServo, double servoPosition, double delayStartTime, boolean isLocationToggle) {
        this.actionServo = actionServo;
        this.servoPosition = servoPosition;
        this.delayStartTime = delayStartTime;
        this.isLocationToggle = isLocationToggle;
    }

    public void executeMotion() {
        if (status != ActionState.COMPLETE) {
            if (actionServo != null) { // If move servo
                actionServo.setPosition(servoPosition);
            } else if (actionMotor != null) {
                actionMotor.setPower(motorPower);
            } else {
                throw new NullPointerException("MotionAction must set either motor or servo");
            }
            status = ActionState.COMPLETE;
        }
    }

    public double getDelayStartTime() {
        return delayStartTime;
    }

    public ActionState getStatus() {
        return status;
    }

    public boolean isLocationToggle() {
        return isLocationToggle;
    }
}
