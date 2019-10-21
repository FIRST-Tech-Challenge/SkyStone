package org.firstinspires.ftc.teamcode.newhardware;

/**
 * Created by User on 11/8/2015.
 */
public class LinearServo extends FXTServo {

    private double position = 1;
    private double min = 0;
    private double max = 1;
    private double length = 2.54;

    public LinearServo(String address) {
        super(address);
    }

    public void out(double speed) {
        if (position + speed > max) {
            position = max;
        } else {
            position += speed;
        }//else
        setPosition(position);
    }//out

    public void in(double speed) {
        if (position - speed < min) {
            position = min;
        } else {
            position -= speed;
        }//else
        setPosition(position);
    }//out


    @Override
    public void setPosition(double mm) {
        if (mm < length && mm >= 0) {
            super.setPosition(mm / length);
        }//if
    }//goToPosition

    public double getMin() {
        return min;
    }//getMin

    public double getMax() {
        return max;
    }

    public void setRange(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public double getLength() {
        return length;
    }
}
