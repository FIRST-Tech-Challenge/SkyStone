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

package org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation;


public class Robot3DPositionIndicator {
    protected double m_X;
    protected double m_Y;
    protected double m_Z;
    protected double m_RotationX;
    protected double m_RotationZ;
    protected double m_RotationY;
    public Robot3DPositionIndicator(double X, double Y, double Z, double RotationX, double RotationY, double RotationZ){
        this.m_X = X;
        this.m_Y = Y;
        this.m_Z = Z;
        this.m_RotationX = XYPlaneCalculations.normalizeDeg(RotationX);
        this.m_RotationY = XYPlaneCalculations.normalizeDeg(RotationY);
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(RotationZ);
    }
    public Robot3DPositionIndicator(Robot2DPositionIndicator Pos2D, double Y, double RotationX, double RotationZ){
        this.m_X = Pos2D.m_X;
        this.m_Y = Y;
        this.m_Z = Pos2D.m_Z;
        this.m_RotationX = XYPlaneCalculations.normalizeDeg(RotationX);
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(RotationZ);
        this.m_RotationY = XYPlaneCalculations.normalizeDeg(Pos2D.m_RotationY);
    }
    public Robot3DPositionIndicator(Robot3DPositionIndicator Pos3D){
        this.m_X = Pos3D.m_X;
        this.m_Y = Pos3D.m_Y;
        this.m_Z = Pos3D.m_Z;
        this.m_RotationX = Pos3D.m_RotationX;
        this.m_RotationY = Pos3D.m_RotationY;
        this.m_RotationZ = Pos3D.m_RotationZ;
    }
    public double getX(){
        return this.m_X;
    }
    public void setX(double X){
        this.m_X = X;
    }
    public double getY(){
        return this.m_Y;
    }
    public void setY(double Y){
        this.m_Y = Y;
    }
    public double getZ(){
        return this.m_Z;
    }
    public void setZ(double Z){
        this.m_Z = Z;
    }
    public double getDistanceToOrigin(){
        return (Math.sqrt(Math.pow(this.getX(),2) + Math.pow(this.getZ(),2) + Math.pow(this.getY(),2)));
    }
    public Robot2DPositionIndicator get2DPosition(){
        return new Robot2DPositionIndicator(this.getX(),this.getZ(),this.getRotationY());
    }
    public double getRotationX(){
        return this.m_RotationX;
    }
    public void setRotationX(double RotationX){
        this.m_RotationX = XYPlaneCalculations.normalizeDeg(RotationX);
    }
    public double getRotationY(){
        return this.m_RotationY;
    }
    public void setRotationY(double RotationY){
        this.m_RotationY = XYPlaneCalculations.normalizeDeg(RotationY);
    }
    public double getRotationZ(){
        return this.m_RotationZ;
    }
    public void setRotationZ(double RotationZ){
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(RotationZ);
    }
    public Robot3DPositionIndicator fromFTCRobotAxisToDarbotsRobotAxis(){
        return XYPlaneCalculations.getDarbotsRobotPosition(this);
    }
    public Robot3DPositionIndicator fromDarbotsRobotAxisToFTCRobotAxis(){
        return XYPlaneCalculations.getFTCRobotPosition(this);
    }
}
