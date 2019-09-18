package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.ToggleButton;

import static android.os.SystemClock.sleep;


/**
 * Created by phanau on 1/22/16.
 * Test REV Robotics RevHub IMU
 */

@Autonomous(name="Test: REV IMU Test 1", group ="Test")
//@Disabled
public class RevGyroTestOp extends OpMode {

    private BNO055IMUHeadingSensor mGyro;
    ToggleButton mButton;

    public RevGyroTestOp() {
    }

    public void init() {
        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mGyro = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mGyro.init(7);  // 7: upright crosswise with REV face forward
        mGyro.setDegreesPerTurn(355.0f);     // appears that's what my IMU does ...
        mGyro.setOpMode(this);      // enable debug output

        // create a toggle-button that cycles through the N possible orientations of the IMU
        mButton = new ToggleButton(false, mGyro.numOrientations(), 0);
    }

    public void loop() {

        // use controller input button X to cycle through various orientations
        if (mButton.process(gamepad1.x))
            mGyro.setOrientation(mButton.value());

        // use controller buttons A and B to increase/decrease degrees per turn correction
        if (gamepad1.a)
            mGyro.setDegreesPerTurn(mGyro.getDegreesPerTurn()+0.1f);
        if (gamepad1.b)
            mGyro.setDegreesPerTurn(mGyro.getDegreesPerTurn()-0.1f);

        telemetry.addData("usage", "press X to set orientation, AB to adjust degrees per turn");
        telemetry.addData("orientation index", mGyro.getOrientation());
        telemetry.addData("degrees per turn", mGyro.getDegreesPerTurn());

        telemetry.addData("corrected heading", mGyro.getHeading());
        telemetry.addData("raw orientation", mGyro.getAngularOrientation());
        telemetry.addData("raw position", mGyro.getPosition());
    }

}
