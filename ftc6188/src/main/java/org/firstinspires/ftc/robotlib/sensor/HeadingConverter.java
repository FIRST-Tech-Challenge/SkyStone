package org.firstinspires.ftc.robotlib.sensor;

public class HeadingConverter
{
    private double lastHeading;
    private double thisHeading;
    private int rotations;

    public void update(double unconvertedHeading)
    {
        thisHeading = ((unconvertedHeading % (2* Math.PI))+2*Math.PI)%(2*Math.PI); //convert to a number in [0, 2pi)
        if (lastHeading > 3*Math.PI/2 && thisHeading < Math.PI/2)
        {
            rotations++;
        }
        else if (thisHeading > 3*Math.PI/2 && lastHeading < Math.PI/2)
        {
            rotations--;
        }
        lastHeading = thisHeading;
    }

    public double getConvertedHeading()
    {
        return thisHeading+2*Math.PI*rotations;
    }
}
