package org.firstinspires.ftc.teamcode;

//import android.graphics.Point;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.HashMap;

public class RobotProfile {
    RobotPosition[]  RED_2 = new RobotPosition[4];
    RobotPosition[]  RED_3 = new RobotPosition[4];
    RobotPosition[]  BLUE_2 = new RobotPosition[4];
    RobotPosition[]  BLUE_3 = new RobotPosition[4];

    public enum StartPosition{
        BLUE_2,
        BLUE_3,
        RED_2,
        RED_3
    }
    HashMap<StartPosition, Point[]> stoneScanPoints;
    class Movement {
        double strifeStopDist;
        double forwardStopDist;
    }
    Movement movement;
    class PIDParam {
        double p;
        double i;
        double d;
    }
    PIDParam headingPID;
    PIDParam distancePID;

    class Point {
        int x;
        int y;
    }

    class HardwareSpec {
        double trackWheelDiameter;   //cm diameter
        double leftRightWheelDist;
        int sliderOrigPos;
        int sliderOutPos;
        int liftOrigPos;
        int liftStoneBase;
        int liftGrabExtra;    // addition high above base before open and lower for grab
        int liftPerStone;     // addition high per stone
        double clampAngleInit;
        double clampAngleReady;
        double clampS1Open;
        double clampS1Close;
        double clampS2Open;
        double clampS2Close;
        double hookS0Open;
        double hookS0Close;
        double hookS1Open;
        double hookS1Close;
        int leftEncodeForwardSign;
        int rightEncoderForwardSign;
        int horizontalEncoderForwardSign;
    }

    HardwareSpec hardwareSpec;

    public void populateInitValue() {
        headingPID = new PIDParam();
        headingPID.p = -0.5;
        headingPID.i = 0.0;
        headingPID.d = 0.0;
        distancePID = new PIDParam();
        distancePID.p = 0.1;
        distancePID.i = 0.0;
        distancePID.d = 0.0;
        movement = new Movement();
        movement.forwardStopDist = 25;
        movement.strifeStopDist = 17;
        Point[] p = new Point[3];
        p[0] = new Point();
        p[0].x= 96;
        p[0].y = 235;
        p[1] = new Point();
        p[1].x= 184;
        p[1].y = 235;
        p[2] = new Point();
        p[2].x= 275;
        p[2].y = 235;
        this.stoneScanPoints = new HashMap<RobotProfile.StartPosition, Point[]>();
        this.stoneScanPoints.put(RobotProfile.StartPosition.RED_2, p);  //leftMost 1st block is [0], 3rd block is[2]

        Point[] p2 = new Point[3];
        p2[0] = new Point();
        p2[0].x= 112;
        p2[0].y = 236;
        p2[1] = new Point();
        p2[1].x= 197;
        p2[1].y = 236;
        p2[2] = new Point();
        p2[2].x= 296;
        p2[2].y = 236;
        this.stoneScanPoints.put(RobotProfile.StartPosition.RED_3, p2);

        Point[] p3 = new Point[3];
        p3[0] = new Point();
        p3[0].x= 536;
        p3[0].y = 231;
        p3[1] = new Point();
        p3[1].x= 441;
        p3[1].y = 231;
        p3[2] = new Point();
        p3[2].x= 338;
        p3[2].y = 231;
        this.stoneScanPoints.put(RobotProfile.StartPosition.BLUE_2, p3);

        Point[] p4 = new Point[3];
        p4[0] = new Point();
        p4[0].x= 527;
        p4[0].y = 231;
        p4[1] = new Point();
        p4[1].x= 430;
        p4[1].y = 231;
        p4[2] = new Point();
        p4[2].x= 326;
        p4[2].y = 231;
        this.stoneScanPoints.put(StartPosition.BLUE_3, p4);

        hardwareSpec = new HardwareSpec();
        hardwareSpec.trackWheelDiameter = 3.8;   //cm diameter
        hardwareSpec.leftRightWheelDist = 37.465;  // cm left right dist
        hardwareSpec.sliderOrigPos = 0;
        hardwareSpec.sliderOutPos = -1350;
        hardwareSpec.liftOrigPos = 0;
        hardwareSpec.liftStoneBase = 133;
        hardwareSpec.liftGrabExtra = 127;    // addition high above base before open and lower for grab
        hardwareSpec.liftPerStone = 341;     // addition high per stone
        hardwareSpec.clampAngleInit = 0.86;
        hardwareSpec.clampAngleReady = 0.53;
        hardwareSpec.clampS1Open = 0.695;
        hardwareSpec.clampS1Close = 0.595;
        hardwareSpec.clampS2Open = 0.40;
        hardwareSpec.clampS2Close = 0.50;
        hardwareSpec.hookS0Open = 0.50;
        hardwareSpec.hookS0Close = 0.85;
        hardwareSpec.hookS1Open = 0.5;
        hardwareSpec.hookS1Close = 0.15;

        hardwareSpec.leftEncodeForwardSign = -1;
        hardwareSpec.rightEncoderForwardSign = -1;
        hardwareSpec.horizontalEncoderForwardSign = -1;

        //for starting at R2, the starting point is [0], the 3 possible sky stone position from left to right are in [1],[2],[3]
        RED_2[0] = new RobotPosition(335, 82, 0);
        RED_2[1] = new RobotPosition(260, 24, 0);
        RED_2[2] = new RobotPosition(260, 30,0);
        RED_2[3] = new RobotPosition(260, 50,0);

        RED_3[0] = new RobotPosition(335, 123, 0);
        RED_3[1] = new RobotPosition(260, 71, 0);
        RED_3[2] = new RobotPosition(260, 91,0);
        RED_3[3] = new RobotPosition(260, 112,0);

        BLUE_2[0] = new RobotPosition(33, 82, 0);
        BLUE_2[1] = new RobotPosition(108, 24, 0);
        BLUE_2[2] = new RobotPosition(108, 30,0);
        BLUE_2[3] = new RobotPosition(108, 50,0);

        BLUE_3[0] = new RobotPosition(335, 123, 0);
        BLUE_3[1] = new RobotPosition(260, 71, 0);
        BLUE_3[2] = new RobotPosition(260, 91,0);
        BLUE_3[3] = new RobotPosition(260, 112,0);

    }

    public void saveToFile(File file) throws FileNotFoundException {
        PrintWriter out = new PrintWriter(file);
        GsonBuilder builder = new GsonBuilder();
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String json = gson.toJson(this);
        out.println(json);
        out.close();
    }

    public static RobotProfile loadFromFile(File file) throws FileNotFoundException {
        Gson gson = new Gson();
        return gson.fromJson(new FileReader(file), RobotProfile.class);
    }

    public static void CVFindBrightPic(){

    }
}
