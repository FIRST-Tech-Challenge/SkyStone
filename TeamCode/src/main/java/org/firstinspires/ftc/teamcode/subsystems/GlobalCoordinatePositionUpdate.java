package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sarthak on 10/30/2018.
 */
public class GlobalCoordinatePositionUpdate implements Runnable {

    private DcMotor verticalLeft, verticalRight, horizontal;
    private Telemetry telemetry;

    private double vlPos, vrPos, hPos;
    private double prevLeft, prevRight, prevHorizontal;

    final double COUNTS_PER_INCH = 307.699557;
    private final double length = 13.25 * COUNTS_PER_INCH;

    private double changeInAngle = 0, angle = 0;
    private double x = 0, y = 0;

    private boolean showTelemetry = true;

    private boolean running;

    public GlobalCoordinatePositionUpdate(DcMotor leftVerticalEncoder, DcMotor rightVerticalWheel, DcMotor horizontalWheel, Telemetry telemetry){
        this.verticalLeft = leftVerticalEncoder;
        this.verticalRight = rightVerticalWheel;
        this.horizontal = horizontalWheel;
        this.telemetry = telemetry;

        running = true;
    }

    @Override
    public void run() {
        while (running){
            //Get Current Positions
            vlPos = verticalLeft.getCurrentPosition();
            vrPos = verticalRight.getCurrentPosition();
            hPos = horizontal.getCurrentPosition();

            double leftChange = vlPos - prevLeft;
            double rightChange = vrPos - prevRight;
            double horizontalChange = hPos - prevHorizontal;

            //Calculate Angle
            changeInAngle = (leftChange - rightChange) / (length);
            angle = ((angle + changeInAngle));

            double p = ((rightChange + leftChange) / 2);
            double n = horizontalChange;
            x = x + (p*Math.sin(angle) + n*Math.cos(angle));
            y = y + -(p*Math.cos(angle) - n*Math.sin(angle));

            prevLeft = vlPos;
            prevRight = vrPos;
            prevHorizontal = hPos;

            if(showTelemetry) {
                telemetry.addData("Orientaion", Math.toDegrees(angle));
                telemetry.addData("X Position", x / COUNTS_PER_INCH);
                telemetry.addData("Y Position", y / COUNTS_PER_INCH);
            }
        }
    }

    public double getXCoord(){
        return x;
    }

    public double getYCoord(){
        return y;
    }

    public double getOrienation(){
        return (Math.toDegrees(angle));
    }

    public void enableTelemetry(){
        showTelemetry = true;
    }

    public void disableTelemetry(){
        showTelemetry = false;
    }

    public void stop(){
        running = false;
    }
}
