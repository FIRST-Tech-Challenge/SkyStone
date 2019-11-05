package org.firstinspires.ftc.teamcode;
/*Now you just need a few more lines of code to make this work. In your robot code, add the following line to your import section.

Code:
import com.qualcomm.robotcore.hardware.I2cDevice;
Add this line to your variable declaration section

Code:
ModernRoboticsI2cColorSensor2 colorx;
Then add these two to your init section.

Code:
I2cDevice colori2c = hardwareMap.i2cDevice.get("colorsensorname");
colorx = new ModernRoboticsI2cColorSensor2(colori2c.getI2cController(),colori2c.getPort());
Finally, to get the color number, use the following code. The second line of this will send it to the driver station so you can see it.

Code:
int cnumber = colorx.colorNumber();
telemetry.addData("Colornumber: ",cnumber);

And there you go! An easy way to implement a color sensor in your code.
We've been using this to detect beacons and its worked great so far. If you have any issues feel free to ask for help.*/
public class MaccaAuto {
}
