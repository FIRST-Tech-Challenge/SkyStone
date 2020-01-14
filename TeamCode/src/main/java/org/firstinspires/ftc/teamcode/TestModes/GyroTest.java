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
    HardwareChassis HC = new HardwareChassis(hardwareMap);
    OmniWheel whee = new OmniWheel(HC);
    HardwareGyro gyro;
    @Override
    public void init() {
        gyro = new HardwareGyro(hardwareMap);
        gyro.init(hardwareMap);
    }

    @Override
    public void loop() {
        this.driveSidewardTime(4000,0.05,1000,this.gyro.imu,this.whee,this);
        telemetry.addData("°",this.gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }

    public void driveSidewardTime(long timeInMillis, double power, double smoothness, BNO055IMU imu, OmniWheel wheel, OpMode op){
        double current = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        long timeStart = System.currentTimeMillis();
        int msStuckinLoopStart = op.msStuckDetectLoop;
        op.msStuckDetectLoop = 1073741824;
        while(timeStart+timeInMillis<System.currentTimeMillis()){
            double offset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle - current;
            wheel.setMotors(0,power,offset/smoothness);
        }
    }

    public void driveSidewardColor(){}

    public void driveSidewardEncoder(){}


}


