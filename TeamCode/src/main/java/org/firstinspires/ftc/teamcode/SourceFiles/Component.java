package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Component {
    private HardwareMap hardwareMap;

    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor claw;

    private Servo leftLatch;
    private Servo rightLatch;

    private ColorSensor colorSensor;

    private boolean isLatched = false;
    private String latchStatus = "Pending";

    // enum variables
    public final int LATCH = 1;
    public final int UNLATCH = 0;

    public final int STOP = 0;
    public final int INTAKE = 1;
    public final int RELEASE = 2;

    public Component(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");

        leftLatch = hardwareMap.servo.get("left latch");
        rightLatch = hardwareMap.servo.get("right latch");

        claw = hardwareMap.dcMotor.get("claw");
    }

    // Accessor/Mutator
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public DcMotor getLeftIntake() {return leftIntake;}
    public DcMotor getRightIntake() {return rightIntake;}
    public DcMotor getClaw() {return claw;}
    public Servo getLeftLatch() {return leftLatch;}
    public Servo getRightLatch() {return rightLatch;}
    public ColorSensor getColorSensor() {return colorSensor;}
    public boolean isLatched() {return isLatched;}
    public String getLatchStatus() {return latchStatus;}

    public void setHardwareMap(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap;}
    public void setLeftIntake(DcMotor leftIntake) {this.leftIntake = leftIntake;}
    public void setRightIntake(DcMotor rightIntake) {this.rightIntake = rightIntake;}
    public void setClaw(DcMotor claw) {this.claw = claw;}
    public void setLeftLatch(Servo leftLatch) {this.leftLatch = leftLatch;}
    public void setRightLatch(Servo rightLatch) {this.rightLatch = rightLatch;}
    public void setColorSensor(ColorSensor colorSensor) {this.colorSensor = colorSensor;}
    public void setLatched(boolean latched) {isLatched = latched;}
    public void setLatchStatus(String latchStatus) {this.latchStatus = latchStatus;}

    // Utilities
    public void latch(int mode) {
        if (mode == LATCH) {
            leftLatch.setPosition(0.5);
            rightLatch.setPosition(0.3);
        } else if (mode == UNLATCH) {
            leftLatch.setPosition(1);
            rightLatch.setPosition(0);
        }
    }

    public void intake(int mode) {
        if (mode == INTAKE) {
            leftIntake.setPower(0.5);
            rightIntake.setPower(-0.5);
        } else if (mode == RELEASE) {
            leftIntake.setPower(-0.2);
            rightIntake.setPower(0.2);
        } else if (mode == STOP){
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    //TODO: add color sensor code
}
