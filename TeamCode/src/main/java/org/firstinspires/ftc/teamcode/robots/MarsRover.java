package org.firstinspires.ftc.teamcode.robots;

import org.firstinspires.ftc.teamcode.newhardware.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by User on 11/3/2015.
 */
public class MarsRover extends Robot {

    private Motor arm;
    private Motor drill;

    public MarsRover(HardwareMap hardware) {
        super();
        drill = new Motor("drill");
        arm = new Motor("arm");
    }

    public void armUp() {
        arm.setPower(0.1);
    }//armUp

    public void armDown() {
        arm.setPower(-0.1);
    }//armDown

    public void stopArm() {
        arm.stop();
    }//stopArm

    public void drillIn() {
        drill.setPower(1);
    }//drillIn

    public void drillOut() {
        drill.setPower(-1);
    }//drillOut

    public void stopDrilling() {
        drill.stop();
    }//stropDrilling

}
