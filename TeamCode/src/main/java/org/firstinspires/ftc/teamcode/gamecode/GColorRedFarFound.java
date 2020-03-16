package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous

public class GColorRedFarFound extends AutoOpMode {
    private VoltageSensor ExpansionHub2_VoltageSensor;
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();
        ColorSensor colorSensor;

        colorSensor = hardwareMap.colorSensor.get("colourLeft");
        ExpansionHub2_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 2");

        waitForStart();


        int RedFoundVal = 50; //May have to change this value. the red colour sensor is already set to be programmed in Joulestest
        telemetry.addData("Status", "initialized");
        joules.FoundationDrop();
        joules.DaffyUp();
        joules.TapeMeasurePush();
        waitForStart();


        joules.SlidesUp();
        sleep(1000);
        joules.SlidesStop();

        joules.DriveForward(0.1);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 100));
        joules.Stop();

        joules.StrafeRight(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 1100));
        joules.Stop();

        sleep(100);

        joules.StrafeLeft(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 500));
        joules.Stop();

        sleep(100);

        joules.DriveBackward(0.2);
        sleep(400);
        joules.Stop();

        joules.DriveForward(0.1);
        sleep(1000);
        joules.Stop();

        clearTimer(1);
        while (opModeIsActive() && colorSensor.red()<RedFoundVal && getMilliSeconds(1)< 4000){
            telemetry.addData("seconds", getMilliSeconds(1)/1000);
            telemetry.addData("red value", colorSensor.red());
            joules.DriveForward(0.001);
        }

        joules.Stop();


        joules.FoundationGrab();
        sleep(2000);
        joules.Stop();

        joules.DriveBackward(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 1800));
        joules.Stop();

        sleep(100);

        joules.StrafeRight(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 800));
        joules.Stop();

        joules.FoundationDrop();
        sleep(2000);
        joules.Stop();

        joules.DriveBackward(0.2);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(),600));
        joules.Stop();

        joules.DriveForward(0.1);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(),100));
        joules.Stop();


//        while(colorSensor.blue() < blueTapeVal){
//            joules.StrafeRight(0.4);
//        }
//        joules.Stop();



        joules.StrafeLeft(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 800));
        joules.Stop();

        joules.StrafeLeft(0.5);
        sleep(800);
        joules.Stop();

        joules.SlidesDown();
        sleep(500);
        joules.SlidesStop();

        joules.DriveForward(0.4);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(),650));
        joules.Stop();

        sleep(100);

        joules.StrafeLeft(0.5);
        sleep(800);
        joules.Stop();



//        for (int y = colorSensor.red(); redTapeVal < y; y = colorSensor.red() ){
//             joules.StrafeRight(0.4);
//         }




    }

}
