package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.HashMap;

public class RobotProfile {
//    RobotPosition[]  RED_2 = new RobotPosition[4];
//    RobotPosition[]  RED_3 = new RobotPosition[4];
//    RobotPosition[]  BLUE_2 = new RobotPosition[4];
//    RobotPosition[]  BLUE_3 = new RobotPosition[4];

    HashMap<StartPosition, Point[]> stoneScanPoints;
    HashMap<StartPosition, RobotPosition> robotStartPoints;
    Movement movement;
    PIDParam headingPID;
    PIDParam distancePID;
    HardwareSpec hardwareSpec;

    public static RobotProfile loadFromFile(File file) throws FileNotFoundException {
        Gson gson = new Gson();
        return gson.fromJson(new FileReader(file), RobotProfile.class);
    }

    public void populateInitValue() {
        headingPID = new PIDParam();
        headingPID.p = 5;
        headingPID.i = 0.05;
        headingPID.d = 0.0;
        distancePID = new PIDParam();
        distancePID.p = 0.5;
        distancePID.i = 0.01;
        distancePID.d = 0.0;
        movement = new Movement();
        movement.forwardStopDist = 25;
        movement.strifeStopDist = 17;
        movement.rotateStopAngle = .1;
        Point[] p = new Point[3];
        p[0] = new Point();
        p[0].x= 238;
        p[0].y = 343;
        p[1] = new Point();
        p[1].x= 297;
        p[1].y = 348;
        p[2] = new Point();
        p[2].x= 383;
        p[2].y = 354;
        this.stoneScanPoints = new HashMap<RobotProfile.StartPosition, Point[]>();
        this.stoneScanPoints.put(RobotProfile.StartPosition.RED_2, p);  //leftMost 1st block is [0], 3rd block is[2]

        Point[] p2 = new Point[3];
        p2[0] = new Point();
        p2[0].x= 249;
        p2[0].y = 346;
        p2[1] = new Point();
        p2[1].x= 319;
        p2[1].y = 350;
        p2[2] = new Point();
        p2[2].x= 408;
        p2[2].y = 358;
        this.stoneScanPoints.put(RobotProfile.StartPosition.RED_3, p2);

        Point[] p3 = new Point[3];
        p3[0] = new Point();
        p3[0].x= 314;
        p3[0].y = 346;
        p3[1] = new Point();
        p3[1].x= 221;
        p3[1].y = 349;
        p3[2] = new Point();
        p3[2].x= 111;
        p3[2].y = 353;
        this.stoneScanPoints.put(RobotProfile.StartPosition.BLUE_2, p3);

        Point[] p4 = new Point[3];
        p4[0] = new Point();
        p4[0].x= 262;
        p4[0].y = 348;
        p4[1] = new Point();
        p4[1].x= 167;
        p4[1].y = 353;
        p4[2] = new Point();
        p4[2].x= 57;
        p4[2].y = 358;
        this.stoneScanPoints.put(StartPosition.BLUE_3, p4);

        this.robotStartPoints = new HashMap<StartPosition, RobotPosition>();
        RobotPosition R2_start = new RobotPosition();
        R2_start.setX(335);
        R2_start.setY(85);
        R2_start.setHeading(Math.PI/2);
        this.robotStartPoints.put(StartPosition.RED_2, R2_start);

        RobotPosition R3_start = new RobotPosition();
        R3_start.setX(335);
        R3_start.setY(144);
        R3_start.setHeading(Math.PI/2);
        this.robotStartPoints.put(StartPosition.RED_3, R3_start);

        RobotPosition B2_start = new RobotPosition();
        B2_start.setX(23);
        B2_start.setY(78);
        B2_start.setHeading(0);
        this.robotStartPoints.put(StartPosition.BLUE_2, B2_start);

        RobotPosition B3_start = new RobotPosition();
        B3_start.setX(23);
        B3_start.setY(138);
        B3_start.setHeading(0);
        this.robotStartPoints.put(StartPosition.BLUE_3,B3_start);

        hardwareSpec = new HardwareSpec();
        hardwareSpec.trackWheelDiameter = 3.8;   //cm diameter
        hardwareSpec.trackWheelCPR = 4000;
        hardwareSpec.leftRightWheelDist = 39.75;  // cm left right dist
        hardwareSpec.sliderOrigPos = 0;
        hardwareSpec.sliderOutPos = 1500;
        hardwareSpec.sliderOutMax = 1825;
        hardwareSpec.sliderOutMin = 1229;
        hardwareSpec.sliderOutMinCM = 3.0;
        hardwareSpec.sliderCountPerCM = 47;
        hardwareSpec.liftOrigPos = 0;
        hardwareSpec.liftStoneBase = 0;
        hardwareSpec.liftHomeReadyPos = 150;
        hardwareSpec.liftHomeGrabPos = 60;
        hardwareSpec.liftGrabExtra = 127;    // addition high above base before open and lower for grab
        hardwareSpec.liftPerStone = 341;     // addition high per stone
        hardwareSpec.clampAngleNormal = 0.88;
        hardwareSpec.clampAngleSide = 0.54;
        hardwareSpec.clampAngleBack = 0; //0.25
        hardwareSpec.clampS1Init = 0.5;
        hardwareSpec.clampS1Open = 0.5; //was open, 0.5
        hardwareSpec.clampS1Close = 0.585; //was close
        hardwareSpec.clampS2Init = 0.5;
        hardwareSpec.clampS2Open= 0.5; //was open, 0.5
        hardwareSpec.clampS2Close = 0.415; //was close
        hardwareSpec.hookS0Open = 0.24;
        hardwareSpec.hookS0Close = 0.66;
        hardwareSpec.hookS1Open = 0.65;
        hardwareSpec.hookS1Close = 0.19;
        hardwareSpec.capStoneUp = 0.34;
        hardwareSpec.capStoneDown = 0.800;
        hardwareSpec.capStoneOther = 0.15;

        hardwareSpec.leftEncodeForwardSign = -1;
        hardwareSpec.rightEncoderForwardSign = -1;
        hardwareSpec.horizontalEncoderForwardSign = 1;

//        //for starting at R2, the starting point is [0], the 3 possible sky stone position from left to right are in [1],[2],[3]
//        RED_2[0] = new RobotPosition(335, 82, Math.PI/2);
//        RED_2[1] = new RobotPosition(260, 24, Math.PI/2);
//        RED_2[2] = new RobotPosition(260, 30,Math.PI/2);
//        RED_2[3] = new RobotPosition(260, 50,Math.PI/2);
//
//        RED_3[0] = new RobotPosition(335, 123, Math.PI/2);
//        RED_3[1] = new RobotPosition(260, 71, Math.PI/2);
//        RED_3[2] = new RobotPosition(260, 91,Math.PI/2);
//        RED_3[3] = new RobotPosition(260, 112,Math.PI/2);
//
//        BLUE_2[0] = new RobotPosition(33, 82, 0);
//        BLUE_2[1] = new RobotPosition(108, 24, 0);
//        BLUE_2[2] = new RobotPosition(108, 30,0);
//        BLUE_2[3] = new RobotPosition(108, 50,0);
//
//        BLUE_3[0] = new RobotPosition(335, 123, 0);
//        BLUE_3[1] = new RobotPosition(260, 71, 0);
//        BLUE_3[2] = new RobotPosition(260, 91,0);
//        BLUE_3[3] = new RobotPosition(260, 112,0);

    }

    public void saveToFile(File file) throws FileNotFoundException {
        PrintWriter out = new PrintWriter(file);
        GsonBuilder builder = new GsonBuilder();
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String json = gson.toJson(this);
        out.println(json);
        out.close();
    }

    public enum StartPosition{
        BLUE_2,
        BLUE_3,
        RED_2,
        RED_3
    }

    class Point {
        int x;
        int y;
    }

    class Movement {
        double strifeStopDist;
        double forwardStopDist;
        double rotateStopAngle;
    }

    class PIDParam {
        double p;
        double i;
        double d;
    }

    class SkyStonePosition {

        double red2SkyStonePosition0_y = 70;
        double red2SkyStonePosition1_y = 56;
        double red2SkyStonePosition2_y = 28;
        double red3SkyStonePosition0_y = 60;
        double red3SkyStonePosition1_y = 40;
        double red3SkyStonePosition2_y = 20;
        double blue2SkyStonePosition0_y = -33;
        double blue2SkyStonePosition1_y = -33;
        double blue2SkyStonePosition2_y = -13;
        double blue3SkyStonePosition0_y = -53;
        double blue3SkyStonePosition1_y = -33;
        double blue3SkyStonePosition2_y = -13;

    }

    class HardwareSpec {
        double trackWheelDiameter;   //cm diameter
        double trackWheelCPR;
        double leftRightWheelDist;
        int sliderOrigPos;
        int sliderOutMax;
        int sliderOutMin;
        double sliderOutMinCM;
        int sliderCountPerCM;
        int sliderOutPos;
        int liftOrigPos;
        int liftStoneBase;
        int liftHomeGrabPos;
        int liftHomeReadyPos;
        int liftGrabExtra;    // addition high above base before open and lower for grab
        int liftPerStone;     // addition high per stone
        double clampAngleNormal;
        double clampAngleSide;
        double clampAngleBack;
        double clampS1Init;
        double clampS1Open;
        double clampS1Close;
        double clampS2Init;
        double clampS2Open;
        double clampS2Close;
        double hookS0Open;
        double hookS0Close;
        double hookS1Open;
        double hookS1Close;
        double capStoneUp; //Original position of the CapStoneServo
        double capStoneDown; //Places the Cap Stone on current stone in the intake
        double capStoneOther; //Delivers the Cap Stone by itself
        int leftEncodeForwardSign;
        int rightEncoderForwardSign;
        int horizontalEncoderForwardSign;
    }
}
