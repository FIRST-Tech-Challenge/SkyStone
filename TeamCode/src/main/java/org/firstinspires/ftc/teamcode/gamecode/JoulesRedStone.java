package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous
public class JoulesRedStone extends AutoOpMode {
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();
        ColorSensor colorSensor;
        ColorSensor colorSensorDown;
        DistanceSensor StoneDist;
        int STONESTATE = 0;
        int STRAFESTATE = 0;

        telemetry.addData("Status", "initialized");


        colorSensor = hardwareMap.colorSensor.get("colour");
        colorSensorDown = hardwareMap.colorSensor.get("ColourDown");
        StoneDist = hardwareMap.get(DistanceSensor.class, "Distance");


        int a = 1;
        int i = 1;
        int o = 1;
        int x = 0;

        int redline = 310; //blue value of blueline coloursensordown.blue
        int rowdist = 300; //when robot detects row of stones distancesensor
        int strafedistlow = 90; //how close robot has to be to stones for the colour sensor to know if its a skystone or a stone distancesensor
        int strafedisthigh = 120;
        int pickupdist = 35;
        int skystonevalue = 55; //normal stone lowest value coloursensor.alpha

        waitForStart();
        while (opModeIsActive()) {
            if (STONESTATE == 0) {
                if (i == 1) {
                    joules.StrafeRight(0.4);
                    sleep(2200);
                    joules.Stop();
                    i += 1;
                }
                joules.DriveBackward(0.5); //go fast
                if (colorSensorDown.blue() > redline) {// if detect blue line
                    STONESTATE = 1;
                    RC.t.addData("Detected Blue Line");
                }
            }

            if (STONESTATE == 1) {// if detect blue line
                RC.t.addData("Stone State", STONESTATE);
                //drive backwards fast
                if (a == 1) {
                    RC.t.addData("little forwards");
                    joules.DriveBackward(0.5);
                    sleep(1000);
                    a = a + 1; //makes this loop happen once
                }
                joules.DriveBackward(0.4);
                if (StoneDist.getDistance(DistanceUnit.MM) < rowdist) {//if detect row of stones
                    STONESTATE = 2;
                }


            }
            if (STONESTATE == 2) { //if detect row  of stones
                RC.t.addData("Row of stones detected Stone State", STONESTATE);
                joules.Stop();
                //drive backwards medium
                if (STRAFESTATE == 0) {
                    joules.Stop();
                    joules.DriveBackward(0.2);
                    if (StoneDist.getDistance(DistanceUnit.MM) > strafedisthigh) {
                        STRAFESTATE = 1;
                    }
                    else if (StoneDist.getDistance(DistanceUnit.MM)< strafedistlow){
                        STRAFESTATE = 2;

                    }else if (colorSensor.alpha()<skystonevalue) { //if detect skystones
                        joules.Stop();
                        STONESTATE = 3;
                    }

                }

                if (STRAFESTATE == 1) {
                    joules.Stop();
                    joules.StrafeRight(0.3);
                    if (StoneDist.getDistance(DistanceUnit.MM) < strafedisthigh) {
                        STRAFESTATE = 0;
                    }
                }
                if (STRAFESTATE == 2){
                    joules.Stop();
                    joules.StrafeLeft(0.3);
                    if (StoneDist.getDistance(DistanceUnit.MM) > strafedistlow) {
                        STRAFESTATE = 0;
                    }
                }

                if (STONESTATE == 3) {
                    joules.StrafeRight(0.3);
                    sleep(500);
                    joules.Stop();
                    STONESTATE = 4;
                }


                 if (STONESTATE == 4){
                    joules.StoneDown();
                    sleep(3300);
                    joules.StoneStop();
                    joules.StrafeLeft(0.3);
                    sleep(1600);
                    joules.Stop();
                    joules.StrafeRight(0.3);
                    sleep(200);
                    joules.Stop();
                    joules.StrafeLeft(0.3);
                    sleep(200);
                    joules.Stop();
                    STONESTATE = 5;}

                if (STONESTATE == 5) {
                    RC.t.addData("blue", colorSensorDown.blue());
                    joules.DriveForward(0.2);
                    STONESTATE = 6;
//                    if (colorSensorDown.blue() > 40) {// if detect blue line
//                        RC.t.addData("blue line");
//                        STONESTATE = 6;}


                }

                if (STONESTATE == 6) {
                    if (o == 1) {
                        joules.DriveForward(0.5);
                        sleep(3000);
                        joules.Stop();
                        joules.StoneUp();
                        sleep(2000);
                        joules.StoneStop();
                        joules.DriveBackward(0.5);
                        sleep(2000);
                        joules.Stop();
                        o += 1;
                    }
                }




            }
        }

    }
}
