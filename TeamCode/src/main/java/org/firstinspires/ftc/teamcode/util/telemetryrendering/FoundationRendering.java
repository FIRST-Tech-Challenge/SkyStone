package org.firstinspires.ftc.teamcode.util.telemetryrendering;

public class FoundationRendering extends TelemetryRender{

    private String telemetryOutput;

    private int xConstraint;
    private int yConstraint;

    //Will be made sure that it only goes on the +s
    //Respect to Foundation Coordinate System
    private int clickerLocationX;
    private int clickerLocationY;

    // 0 - 0 degrees, 1 - 90 degrees, 2 - 180 degrees, 3 - 270 degrees
    private int clickerDirection;

    public FoundationRendering(int xConstraint, int yConstraint){

        this.xConstraint = xConstraint;
        this.yConstraint = yConstraint;

        StringBuilder stringBuilder = new StringBuilder();
        for(int j = 0; j < yConstraint; j++){
            for(int i = 0; i < xConstraint; i++){
                stringBuilder.append("+");
                if(!(i + 1 == xConstraint)){
                    stringBuilder.append(" ");
                } else {
                    stringBuilder.append("\n");
                }
            }
        }

        clickerDirection = 0;

        clickerLocationX = 1;
        clickerLocationY = 1;

        telemetryOutput = stringBuilder.toString();
    }

    public void setTelemetryOutput(String telemetryOutput){
        this.telemetryOutput = telemetryOutput;
    }

    public void setXConstraint(int xConstraint){
        this.xConstraint = xConstraint;
    }

    public void setYConstraint(int yConstraint){
        this.yConstraint = yConstraint;
    }

    public String getTelemetryOutput(){
        return telemetryOutput;
    }



    public void update(){
        StringBuilder stringBuilder = new StringBuilder();
        for(int j = 0; j < yConstraint; j++){
            for(int i = 0; i < xConstraint; i++){
                stringBuilder.append("+");
                if(!(i + 1 == xConstraint)){
                    stringBuilder.append(" ");
                } else {
                    stringBuilder.append("\n");
                }
            }
        }
    }

}
