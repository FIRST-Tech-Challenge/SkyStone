package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;

//by Lena and Simeon

@Disabled
@TeleOp(name = "GyroPlayerTest")

public class GyroPlayerTest extends OpMode {
    HardwareGyro gyro;
    OmniWheel oWheel;
    HardwareChassis robot;
    ColorTools colorTools;
    OrientationTools oTool;

    double smoothness;
    double offset;
    double posTodrive;

    double power_X;
    double power_Y;



    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        oWheel = new OmniWheel(robot);
        oTool = new OrientationTools(robot,hardwareMap,null);
        gyro = new HardwareGyro(hardwareMap);
        gyro.init(hardwareMap);

        smoothness = 200;
        posTodrive = oTool.getDegree360(gyro.imu);
        offset = posTodrive -oTool.getDegree360(gyro.imu);

        super.msStuckDetectLoop = 10000000;
    }

    @Override
    public void loop() {
        oWheel.setMotors(power_Y*0.5,power_X*0.5,offset/smoothness);
        //-----------------------------------------------------------------------------------
        offset = oTool.getDegree360(gyro.imu) - posTodrive;
        if(gamepad1.right_bumper) {
            while(gamepad1.right_bumper){
                oWheel.setMotors(0,0,gamepad1.right_trigger);
            }
            posTodrive = oTool.getDegree360(gyro.imu);
        }
        if(gamepad1.left_bumper) {
            while(gamepad1.left_bumper){
                oWheel.setMotors(0,0,-gamepad1.right_trigger);
            }
            posTodrive = oTool.getDegree360(gyro.imu);
        }
        //-----------------------------------------------------------------------------------
        power_X = (double)(gamepad1.right_trigger*(gamepad1.left_stick_x/Math.max(Math.abs(gamepad1.left_stick_x),Math.abs(gamepad1.left_stick_y)+0.001)));
        power_Y = -(double)(gamepad1.right_trigger*(gamepad1.left_stick_y/Math.max(Math.abs(gamepad1.left_stick_x),Math.abs(gamepad1.left_stick_y)+0.001)));
        //------------------------------------------------------------------------------------------------------------


















        telemetry.addData("offset",offset);
        telemetry.addData("offset/smoothness",offset/smoothness);
        telemetry.addData("posTodrive",posTodrive);
        telemetry.addData("r_trigger",gamepad1.right_trigger);
        telemetry.addData("power_X",power_X);
        telemetry.addData("power_Y",power_Y);

        telemetry.addData("x",gamepad1.left_stick_x);
        telemetry.addData("y",gamepad1.left_stick_y);
        telemetry.addData("isNaN_x",gamepad1.left_stick_x != 0.0/0.0);
        telemetry.addData("isNaN_y",Float.isNaN(gamepad1.left_stick_y));
        telemetry.update();
    }
}


