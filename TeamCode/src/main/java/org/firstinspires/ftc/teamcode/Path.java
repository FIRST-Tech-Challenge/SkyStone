package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Arrays;
public class Path{
    ArrayList<Coordinate> list = new ArrayList<Coordinate>();
    double slope;
    double constant;
    public Path(Coordinate start, Coordinate end){
        list.add(start);
        list.add(end);
    }
    public double getDesiredY(double x){
      return slope*x + constant;
    }
    public double getDesiredX(double y){
      return (y- constant)/slope;
    }

    /**
     * returns coordinate of closest proximity on the line
     * @param curr
     * @return
     */
    public Coordinate getDesiredCoordinate(Coordinate curr){
        return list.get(1);
    }
    public void addPoint(Coordinate n){
        list.add(n);
    }
}
