package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DemoArm {
    private DcMotor armLift;

    public DemoArm(HardwareMap hardwareMap){
        armLift = hardwareMap.get(DcMotor.class, DemoComponentNames.ARM_LIFT);
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
