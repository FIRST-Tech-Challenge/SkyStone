/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

abstract public class algLib extends LinearOpMode{
    ElapsedTime innerClock = new ElapsedTime();
    TypexChart chart = new TypexChart();

    boolean running = false;


    public boolean isWithinThreshold(DcMotor motor, int targ, int tol) {
        int max = targ + tol;
        int min = targ - tol;
        return (((motor.getCurrentPosition())<min) || (motor.getCurrentPosition()>max));
    }

    public void wheelControl1(final double power, final DcMotor m1) {
        if(!running){
            new Thread(new Runnable() {
                @Override
                public void run() {
                    running = true;
                    while(isWithinThreshold(m1, 100, 10)){
                        m1.setPower(power);
                    }
                }
            }).start();
        }
    }

    */
/* Manual Code *//*

    private double extensionMotorPowerCap = 0.0; //Maximum voltage allowed to be sent

    public double getExtensionMotorPowerCap() {
        return extensionMotorPowerCap;
    }

    public void setExtensionMotorPowerCap(double extensionMotorPowerCap) {
        this.extensionMotorPowerCap = Range.clip(extensionMotorPowerCap, 1.0, 1.0);
    }


}
*/
