package org.firstinspires.ftc.teamcode.Skystone.Auto.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionState;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class MotionAction {
    Robot robot;
    private DcMotor actionMotor;
    private Servo actionServo;

    private double servoPosition; // servo
    private double motorPower; // motor

    private String motorPosition;

    private double delayStartTime;

    private ActionState status = ActionState.PENDING;

    public MotionAction(DcMotor actionMotor, double motorPower, double delayStartTime, Robot robot) {
        this.actionMotor = actionMotor;
        this.motorPower = motorPower;
        this.delayStartTime = delayStartTime;
        this.robot = robot;

    }

    public MotionAction(DcMotor actionMotor, double motorPower, int motorPosition, double delayStartTime, Robot robot) {
        this.actionMotor = actionMotor;
        this.motorPower = motorPower;
        this.delayStartTime = delayStartTime;
        this.motorPosition = Integer.toString(motorPosition);
        this.robot = robot;
    }

    public MotionAction(Servo actionServo, double servoPosition, double delayStartTime, Robot robot) {
        this.actionServo = actionServo;
        this.servoPosition = servoPosition;
        this.delayStartTime = delayStartTime;
        this.robot = robot;

    }

    public void executeMotion() {
        if (status != ActionState.COMPLETE) {
            if (actionServo != null) { // If move servo
                actionServo.setPosition(servoPosition);
            } else if (actionMotor != null && actionMotor.equals(robot.getOuttakeSpool())) {
                int spoolPosition = robot.getOuttakeSpool().getCurrentPosition();
                double spoolPower;
                if (Math.abs(spoolPosition - Integer.parseInt(motorPosition)) < 25) {
                    spoolPower = 0.175;
                    status = ActionState.COMPLETE;

                } else if (spoolPosition < Integer.parseInt(motorPosition)) {
                    spoolPower = 0.8;
                } else {
                    spoolPower = -0.4;
                }
                robot.getOuttakeSpool().setPower(spoolPower);
                robot.getOuttakeSpool2().setPower(spoolPower);
            } else if (actionMotor != null) {
                actionMotor.setPower(motorPower);
                status = ActionState.COMPLETE;
            } else {
                throw new NullPointerException("MotionAction must set either motor or servo");
            }
            if (actionServo != null) {
                status = ActionState.COMPLETE;
            }
        }
    }

    public double getDelayStartTime() {
        return delayStartTime;
    }

    public ActionState getStatus() {
        return status;
    }
}
