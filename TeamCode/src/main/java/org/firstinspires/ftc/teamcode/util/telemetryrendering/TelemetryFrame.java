package org.firstinspires.ftc.teamcode.util.telemetryrendering;

import android.annotation.TargetApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@TargetApi(24)
public class TelemetryFrame {

    private Telemetry telemetry;
    private HashMap<String, Object> telemetryItems;

    public TelemetryFrame(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void setToThisFrame(){
        telemetry.clearAll();
        for(String s : telemetryItems.keySet()){
            telemetry.addData(s,telemetryItems.get(s));
        }
    }

    public void addTelemetryItem(String caption, Object value){
        telemetryItems.put(caption,value);
    }

    public void updateTelemetryItem(String caption, Object value){
        telemetryItems.replace(caption,value);
    }

}
