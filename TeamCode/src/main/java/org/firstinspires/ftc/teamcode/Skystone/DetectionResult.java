package org.firstinspires.ftc.teamcode.Skystone;

public class DetectionResult {

    private int count = 0;
    private float confidence;
    private Vision.Location location;

    public int getCount() {
        return count;
    }

    public void setCount(int count) {
        this.count = count;
    }

    public float getConfidence() {
        return confidence;
    }

    public void setConfidence(float confidence) {
        this.confidence = confidence;
    }

    public Vision.Location getLocation() {
        return location;
    }

    public void setLocation(Vision.Location location) {
        this.location = location;
    }

    public DetectionResult(Vision.Location location){
        this.location = location;
    }

    public void incrementConfidence(float confidence){
        count++;
        confidence+= confidence;
    }

    public float avgConfidence(){
        if (count != 0){
            return confidence/count;
        }
        return 0;
    }

}
