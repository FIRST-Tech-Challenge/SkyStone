package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement.Localization.Odometer;

public class Timer {

    private ElapsedTime elapsedTime = new ElapsedTime();
    private LinearOpMode opMode;
    private Odometer odometer;

    public Timer(LinearOpMode opMode, Odometer odometer){
        this.opMode = opMode;
        this.odometer = odometer;

    }

    public void start(){
        elapsedTime.reset();
    }

    public void waitMillis(double millis){
        double initialTime = elapsedTime.milliseconds();
        double endTime = millis+initialTime;
        while(opMode.opModeIsActive()){
            odometer.update();
            if(elapsedTime.milliseconds() > endTime){
                break;
            }
        }
    }

}
