package org.firstinspires.ftc.teamcode;

public class CONSTANTS {
    public double OPENPOSITION = 0.9;
    public double CLOSEPOSITION = 0.1;
    public double STALKPOWER = 0.3, STRAFEPOWER = 0.7, HALFPOWER = 0.5;

    private boolean speedToggle;
    private boolean positionToggle;

    public double speedMultip = 1;

    public boolean isPositionToggle() {
        return positionToggle;
    }

    public void setPositionToggle(boolean positionToggle) {
        this.positionToggle = positionToggle;
    }

    public boolean isSpeedToggle() {
        return speedToggle;
    }

    public void setSpeedToggle(boolean speedToggle) {
        this.speedToggle = speedToggle;
    }
}
