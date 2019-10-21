package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import android.util.Log;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;
import org.firstinspires.ftc.teamcode.util.MathUtils;

/**
 * Created by FIXIT on 16-10-08.
 */
public class TrackBall {

    private DcMotor xEnc;
    private DcMotor yEnc;

    private Point absoluteFieldCoord = new Point(0, 0);
    private Point lastTiks;

    public TrackBall(String xAddr, String yAddr) {
        xEnc = RC.h.dcMotor.get(xAddr);
        yEnc = RC.h.dcMotor.get(yAddr);

        lastTiks = getEncTiks();
    }//TrackBall

    //switch encoder ports
    //and reverse the values
    public Point getEncTiks() {
        return new Point(-xEnc.getCurrentPosition(), -yEnc.getCurrentPosition());
    }//getEncTiks

    public void setAbsoluteCoord(Point coord) {
        this.absoluteFieldCoord = coord;
    }//setAbsoluteCoord

    public Point getAbsoluteCoord() {
        return absoluteFieldCoord;
    }//void

    public void addAbsoluteCoordinateRunnable (final AdafruitBNO055IMU imu) {

        TaskHandler.addLoopedTask("TrackBall.ABSOLUTECOORDINATE", new Runnable() {
            @Override
            public void run() {
                synchronized (imu) {
                    updateAbsolutePoint(-imu.getAngularOrientation().firstAngle);
                }//synchronized
            }//run
        }, 5);//TaskHandler

    }//addAbsoluteCoordinateRunnable

    public void updateAbsolutePoint(double robotAngle) {

        robotAngle = MathUtils.cvtAngleToNewDomain(robotAngle);

        Point delta = getEncTiks().subtract(lastTiks);
        lastTiks = getEncTiks();

        Log.i("EncTiks", delta.toString() + ", " + getEncTiks());
        Log.i("Angle", robotAngle + "");

        /*
        y = y * cos(robotAngle) + x * cos(robotAngle + 90)
        x = y * sin(robotAngle) + x * sin(robotAngle + 90)
         */



        double yChange = delta.y * Math.cos(Math.toRadians(robotAngle)) + delta.x * Math.cos(Math.toRadians(robotAngle + 90));
        double xChange = delta.y * Math.sin(Math.toRadians(robotAngle)) + delta.x * Math.sin(Math.toRadians(robotAngle + 90));

//        double yChange = delta.y * Math.cos(Math.toRadians(robotAngle)) - delta.x * Math.sin(Math.toRadians(robotAngle));
//        double xChange = delta.y * Math.sin(Math.toRadians(robotAngle)) + delta.x * Math.cos(Math.toRadians(robotAngle));

        Point change = new Point(xChange, yChange);

        Log.i("Tik Change", change.toString());

        absoluteFieldCoord = absoluteFieldCoord.add(change);
    }//updateAbsolutePoint

    public static class Point {

        private double x;
        private double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public void setX(double x) {
            this.x = x;
        }//setX

        public void setY(double y) {
            this.y = y;
        }//setY

        public double getX() {
            return x;
        }//getX

        public double getY() {
            return y;
        }//getY

        public Point subtract (Point pt) {
            return new Point(this.x - pt.x, this.y - pt.y);
        }

        public Point add(Point pt) {
            return new Point(this.x + pt.x, this.y + pt.y);
        }

        public Point multiply(double scalar) {
            return new Point(this.x * scalar, this.y * scalar);
        }

        public double hypot() {
            return Math.hypot(x, y);
        }//hypot

        public double atan() {
            return Math.toDegrees(Math.atan2(y, x));
        }//atan

        public double acot() {
            return Math.toDegrees(Math.atan2(x, y));
        }//acot

        public String toString() {
            return "{" + this.x + ", " + this.y + "}";
        }
    }//Point
}//TrackBall
