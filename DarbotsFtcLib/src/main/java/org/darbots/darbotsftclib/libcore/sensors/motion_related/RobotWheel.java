/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.sensors.motion_related;


import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;

public class RobotWheel {
    private double m_Radius;
    private Robot2DPositionIndicator m_OnRobotPosition;

    public RobotWheel(Robot2DPositionIndicator OnRobotPosition, double radius){
        this.m_OnRobotPosition = OnRobotPosition;
        this.m_Radius = radius;
    }
    public Robot2DPositionIndicator getOnRobotPosition(){
        return this.m_OnRobotPosition;
    }
    public void setOnRobotPosition(Robot2DPositionIndicator OnRobotPosition){
        this.m_OnRobotPosition = OnRobotPosition;
    }
    public double getRadius(){
        return this.m_Radius;
    }
    public void setRadius(double Radius){
        this.m_Radius = Radius;
    }
    public double getCircumference(){
        return this.m_Radius * 2 * Math.PI;
    }
    public double getXPerCounterClockwiseDistance(){
        return -Math.cos(Math.toRadians(this.m_OnRobotPosition.getRotationY()));
    }
    public double getZPerCounterClockwiseDistance(){
        return -Math.sin(Math.toRadians(this.m_OnRobotPosition.getRotationY()));
    }
    public double getDistanceFromCenterOfRobot(){
        return this.m_OnRobotPosition.getDistanceToOrigin();
    }
}
