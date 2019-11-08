package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public abstract class Robot2DPositionTracker {
    private Robot2DPositionIndicator m_InitialPos;
    private Robot2DPositionIndicator m_CurrentPos;
    private double m_RobotWidth;
    private double m_RobotHeight;
    public Robot2DPositionTracker(Robot2DPositionIndicator initialPosition){
        this.m_InitialPos = new Robot2DPositionIndicator(initialPosition);
        this.m_CurrentPos = new Robot2DPositionIndicator(initialPosition);
    }
    public Robot2DPositionTracker(Robot2DPositionIndicator initialPosition, double RobotWidth, double RobotHeight){
        this.m_InitialPos = new Robot2DPositionIndicator(initialPosition);
        this.m_CurrentPos = new Robot2DPositionIndicator(initialPosition);
        this.m_RobotWidth = RobotWidth;
        this.m_RobotHeight = RobotHeight;
    }
    public Robot2DPositionTracker(Robot2DPositionTracker oldTracker){
        this.m_InitialPos = new Robot2DPositionIndicator(oldTracker.m_InitialPos);
        this.m_CurrentPos = new Robot2DPositionIndicator(oldTracker.m_CurrentPos);
        this.m_RobotWidth = oldTracker.m_RobotWidth;
        this.m_RobotHeight = oldTracker.m_RobotHeight;
    }

    public Robot2DPositionIndicator getInitialPos(){
        return this.m_InitialPos;
    }
    public void setInitialPos(Robot2DPositionIndicator initialPos){
        this.m_InitialPos = initialPos;
    }
    public Robot2DPositionIndicator getCurrentPosition(){
        return this.m_CurrentPos;
    }
    public void setCurrentPosition(Robot2DPositionIndicator currentPosition){
        this.m_CurrentPos = currentPosition;
    }
    public double getRobotWidth(){
        return this.m_RobotWidth;
    }
    public void setRobotWidth(double RobotWidth){
        this.m_RobotWidth = RobotWidth;
    }
    public double getRobotHeight(){
        return this.m_RobotHeight;
    }
    public void setRobotHeight(double Height){
        this.m_RobotHeight = Height;
    }
    public Robot2DPositionIndicator getRobotAxisLeftTopExtremePoint(){
        return new Robot2DPositionIndicator(-m_RobotWidth / 2,m_RobotHeight/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisRightTopExtremePoint(){
        return new Robot2DPositionIndicator(m_RobotWidth/2,m_RobotHeight/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisLeftBottomExtremePoint(){
        return new Robot2DPositionIndicator(-m_RobotWidth/2,-m_RobotHeight/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisRightBottomExtremePoint(){
        return new Robot2DPositionIndicator(m_RobotWidth/2,-m_RobotHeight/2,0);
    }
    public Robot2DPositionIndicator fieldAxisFromRobotAxis(Robot2DPositionIndicator RobotAxisPoint){
        return XYPlaneCalculations.getAbsolutePosition(this.getCurrentPosition(),RobotAxisPoint);
    }
    public Robot2DPositionIndicator robotAxisFromFieldAxis(Robot2DPositionIndicator FieldAxisPoint){
        return XYPlaneCalculations.getRelativePosition(this.getCurrentPosition(),FieldAxisPoint);
    }
    public void offsetPosition(Robot2DPositionIndicator offsetPosition) {
        Robot2DPositionIndicator currentPosition = this.getCurrentPosition();
        if(offsetPosition.getX() != 0)
            currentPosition.setX(currentPosition.getX() + offsetPosition.getX());
        if(offsetPosition.getZ() != 0)
            currentPosition.setZ(currentPosition.getZ() + offsetPosition.getZ());
        if(offsetPosition.getRotationY() != 0)
            currentPosition.setRotationY(currentPosition.getRotationY() + offsetPosition.getRotationY());
    }
}
