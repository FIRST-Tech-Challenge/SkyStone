package org.firstinspires.ftc.teamcode;

public class DriverOptions {
    private int delay;
    private String startingPositionModes;
    private String parking;
    private String deliverRoute;
    private String moveFoundation;
    private String isParkOnly;
    private String stoneOptions;
    private boolean isFirstBlockByWall;

    public String getStartingPositionModes() {
        return startingPositionModes;
    }

    public void setStartingPositionModes(String startingPositionModes) {
        this.startingPositionModes = startingPositionModes;
    }

    public String getParking() {
        return parking;
    }

    public void setParking(String parking) {
        this.parking = parking;
    }

    public String getDeliverRoute() {
        return deliverRoute;
    }

    public void setDeliverRoute(String deliverRoute) {
        this.deliverRoute = deliverRoute;
    }

    public String getMoveFoundation() {
        return moveFoundation;
    }

    public void setMoveFoundation(String moveFoundation) {
        this.moveFoundation = moveFoundation;
    }

    public String getIsParkOnly() {
        return isParkOnly;
    }

    public void setIsParkOnly(String isParkOnly) {
        this.isParkOnly = isParkOnly;
    }

    public String getStoneOptions() {
        return stoneOptions;
    }

    public boolean getIsFirstBlockByWall(){return isFirstBlockByWall;}

    public void setStoneOptions(String stoneOptions) { this.stoneOptions = stoneOptions; }

    public void setIsFirstBlockByWall(String isFirstBlockByWall) {
        this.isFirstBlockByWall = isFirstBlockByWall.equals("yes");

    }

    public int getDelay() {
        return delay;
    }

    public void setDelay(int delay) {
        this.delay = delay;
    }
}
