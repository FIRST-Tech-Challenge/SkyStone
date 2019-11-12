package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

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

@Autonomous(name = MaccaAuto)

public class MaccaAuto extends  {
    DcMotorEx front_left, front_right, back_left, back_right;

    @Override
    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.dcMotor.get("front_left"); // Port 0
        front_right = hardwareMap.dcMotor.get("front_right"); // Port 1
        back_left = hardwareMap.dcMotor.get("back_left"); // Port 2
        back_right = hardwareMap.dcMotor.get("back_right"); // Port 3

        //PID Coefficients for tuning
        PIDCoefficients tunedConstants = new PIDCoefficients(0.0025, 0.1, 0.2);
        front_left.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        front_right.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        back_left.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        back_right.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);



        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorTargets(1120, 1120, 1120, 1120);

        front_left.setPower(1);
        front_right.setPower(1);
        back_left.setPower(1);
        back_right.setPower(1);
        while (front_left.isBusy()) {
            Thread.yield();
        }
        front_left.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        front_left.setMode(mode);
        front_right.setMode(mode);
        back_left.setMode(mode);
        back_right.setMode(mode);
    }

    private void setMotorTargets(int tlTarget, int trTarget, int blTarget, int brTarget) {
        front_left.setTargetPosition(trTarget);
        front_right.setTargetPosition(tlTarget);
        back_left.setTargetPosition(blTarget);
        back_right.setTargetPosition(brTarget);
    }

}
