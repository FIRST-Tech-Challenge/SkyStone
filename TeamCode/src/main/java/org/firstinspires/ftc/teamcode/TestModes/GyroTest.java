package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

@TeleOp(name="testGyro")
public class GyroTest extends OpMode {
    HardwareChassis HC;
    OmniWheel whee;
    HardwareGyro gyro;
    @Override
    public void init() {
        HC = new HardwareChassis(hardwareMap);
        gyro = new HardwareGyro(hardwareMap);
        gyro.init(hardwareMap);
        whee = new OmniWheel(HC);
        this.msStuckDetectLoop = 1073741824;
    }

    @Override
    public void loop() {
        this.driveSidewardTime(8000,0.1,100,this.gyro.imu,this.whee,this);
    }

    public void driveSidewardTime(long timeInMillis, double power, double smoothness, BNO055IMU imu, OmniWheel wheel, OpMode op){
        double current = this.getDegree360();
        long timeStart = System.currentTimeMillis();
        int msStuckinLoopStart = op.msStuckDetectLoop;
        op.msStuckDetectLoop = 1073741824;
            while (timeStart + timeInMillis > System.currentTimeMillis()) {
                double offset = this.getDegree360() - current;
                wheel.setMotors(0, power, offset / smoothness);
            }

        op.msStuckDetectLoop = msStuckinLoopStart;
    }

    public void driveSidewardColor(){}

    public void driveSidewardEncoder(){}

    public double getDegree360(){
        return 180+this.gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
    }


}


