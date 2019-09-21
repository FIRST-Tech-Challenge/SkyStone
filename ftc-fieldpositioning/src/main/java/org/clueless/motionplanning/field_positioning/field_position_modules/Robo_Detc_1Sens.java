package org.clueless.motionplanning.field_positioning.field_position_modules;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by DrZ on 11/30/18.
 */

public class Robo_Detc_1Sens {
    private DistanceSensor sensorRange1;
    double Cur;
    double Prev;
    boolean Blocked = true;
    double x;
    double difr;
    double prevr;
    boolean act;


    public Robo_Detc_1Sens(){
        x = sensorRange1.getDistance(DistanceUnit.CM);

    }

    public void calcDs(double x, double prev, double cur){ // ????????
        this.x = x;
        this.Cur = cur;
        this.Prev = prev;
        while(x != 0){// x != is equal to// distancesensor is on.
            if(Prev != 0){
                x = cur;
                difr = cur - prev/prev;
                act = true;
                if(prevr - 10 > difr){
                    Blocked = true;
                }else{
                    Blocked = false;
                }
            }else{
                act = false;
                break;
            }
            difr = prevr;
            cur = Prev;

        }

    }
    public boolean getRobostus(){
        return Blocked;
    }

}
