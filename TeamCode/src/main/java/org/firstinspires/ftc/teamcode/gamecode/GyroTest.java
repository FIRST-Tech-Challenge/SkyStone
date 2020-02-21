package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

import java.lang.ref.Reference;

@Autonomous

public class GyroTest extends AutoOpMode {
    private VoltageSensor ExpansionHub2_VoltageSensor;
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();

        ExpansionHub2_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 2");



        telemetry.addData("Status", "initialized");

        waitForStart();
        Orientation orient = joules.imu.getAngularOrientation();

        //return new double[] {orient.firstAngle, orient.secondAngle, orient.thirdAngle};


        while(opModeIsActive()){
            telemetry.addData("imu", joules.imu.getAngularOrientation());

        }






    }

//    public double[] getIMUAngle() {
//        Orientation orient = joules.imu.getAngularOrientation();
//
//        return new double[] {-orient.firstAngle, -orient.secondAngle, -orient.thirdAngle};
//    }//getIMUAngle
//
//    public double getAngle(){
//        return getIMUAngle()[0];
//    }

}