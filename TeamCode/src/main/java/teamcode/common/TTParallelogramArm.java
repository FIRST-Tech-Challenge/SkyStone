package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TTParallelogramArm {
    private Servo claw;
    private DcMotor elbowJoint;

    //set at 90 degrees
    public void toPresetA(double power){

    }


    public Servo getClaw(){
        return claw;
    }
    public DcMotor getElbowJoint(){
        return elbowJoint;
    }
}
