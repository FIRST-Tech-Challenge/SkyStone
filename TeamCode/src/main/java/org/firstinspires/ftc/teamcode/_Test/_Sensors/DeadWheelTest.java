package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;


/**
 * Created by phanau on 3/20/19.
 * Test deadwheel consisting of a Tetrix motor encoder connected to an encode input of a RevHub
 */
@Autonomous(name="Test: Deadwheel Sensor Test 1", group ="Test")
//@Disabled
public class DeadWheelTest extends OpMode {

    DcMotor mMotor;

    public DeadWheelTest() {
    }

    public void init() {
        // get hardware
        AutoLib.HardwareFactory mf = new AutoLib.RealHardwareFactory(this);
        mMotor = mf.getDcMotor("bl");       // motor 0 in reverseRatbot configuration
    }

    public void loop() {
        // log data to DriverStation
        telemetry.addData("encoder: ", mMotor.getCurrentPosition());
    }

    public void stop() {}

}
