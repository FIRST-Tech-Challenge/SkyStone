package DemoCodes.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import DemoCodes.common.DemoComponentNames;

public class DemoArm {
    private DcMotor armLift;
    private Servo claw;

    public DemoArm(HardwareMap hardwareMap){
        armLift = hardwareMap.get(DcMotor.class, DemoComponentNames.ARM_LIFT);
        claw = hardwareMap.get(Servo.class, DemoComponentNames.CLAW);
    }

    public void raise(){
        armLift.setPower(0.5);
    }

    public void lower(){
        armLift.setPower(-0.5);
    }

    public void stop(){
        armLift.setPower(0);
    }
}