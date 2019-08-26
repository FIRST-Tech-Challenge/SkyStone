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


public class Robot2DPositionIndicator {
    protected double m_X;
    protected double m_Z;
    protected double m_RotationY;
    public Robot2DPositionIndicator(double X, double Z, double YRotation){
        this.m_X = X;
        this.m_Z = Z;
        this.m_RotationY = XYPlaneCalculations.normalizeDeg(YRotation);
    }
    public Robot2DPositionIndicator(Robot2DPositionIndicator Pos2D){
        this.m_X = Pos2D.m_X;
        this.m_Z = Pos2D.m_Z;
        this.m_RotationY = Pos2D.m_RotationY;
    }
    public double getX(){
        return this.m_X;
    }
    public void setX(double X){
        this.m_X = X;
    }
    public double getZ(){
        return this.m_Z;
    }
    public void setZ(double Z){
        this.m_Z = Z;
    }
    public double getDistanceToOrigin(){
        return (Math.sqrt(Math.pow(this.getX(),2) + Math.pow(this.getZ(),2)));
    }
    public double getRotationY(){
        return this.m_RotationY;
    }
    public void setRotationY(double RotationY){
        this.m_RotationY = XYPlaneCalculations.normalizeDeg(RotationY);
    }
}
