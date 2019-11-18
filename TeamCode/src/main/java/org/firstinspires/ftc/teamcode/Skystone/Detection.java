package org.firstinspires.ftc.teamcode.Skystone;

public class Detection {
    int position;
    float value;

    double numDetections;

    Detection(){
        position = 0;
        value = 0;
        numDetections = 0;
    }

    Detection(int position, float value, double numDetections){
        this.position = position;
        this.value = value;
        this.numDetections = numDetections;
    }

    public int getPosition(){
        return position;
    }
    public float getValue(){
        return value;
    }
    public double getNumDetections() {
        return numDetections;
    }
}
