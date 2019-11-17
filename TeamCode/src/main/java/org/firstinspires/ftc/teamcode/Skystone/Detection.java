package org.firstinspires.ftc.teamcode.Skystone;

public class Detection {
    int position;
    float value;
    Detection(){
        position = 0;
        value = 0;
    }
    Detection(int position, float value){
        this.position = position;
        this.value = value;
    }
    public int getPosition(){
        return position;
    }
    public float getValue(){
        return value;
    }
}
