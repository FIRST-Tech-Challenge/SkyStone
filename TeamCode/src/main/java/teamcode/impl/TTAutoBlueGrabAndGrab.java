package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import teamcode.common.BoundingBox2D;
import teamcode.common.League1TTArm;
import teamcode.common.TTDriveSystem;
import teamcode.common.TTOpMode;
import teamcode.common.TTVision;
import teamcode.common.Vector2;

@Autonomous(name = "TT Auto Blue Grab And Grab")
public class TTAutoBlueGrabAndGrab extends TTOpMode {

    /**
     * A bounding box which is used to see if a skystone is in the center of the camera's view.
     */
    private static final BoundingBox2D SKYSTONE_BOUNDING_BOX = new BoundingBox2D(0, 0, 720, 1280);

    private TTDriveSystem driveSystem;
    private League1TTArm arm;
    private TTVision vision;
    private int skystonePos;

    @Override
    protected void onInitialize() {
        driveSystem = new TTDriveSystem(hardwareMap);
        arm = new League1TTArm(hardwareMap);
        vision = new TTVision(hardwareMap);
        vision.enable();

    }

    @Override
    protected void onStart() {
        initArm();
        skystonePos = locateSkystone456();
        if(skystonePos == 6) {
            telemetry.addData("stone number", 6);
            telemetry.update();
            grabSkyStone(6);
            moveAndGrabStone(3);
            arm.openClaw();
            driveSystem.lateral(-10, 0.6);
        } else if(skystonePos == 5){
            telemetry.addData("stone number", 5);
            telemetry.update();
            grabSkyStone(5);
            moveAndGrabStone(2);
            arm.openClaw();
            driveSystem.lateral(-10, 0.6);
        } else {
            telemetry.addData("sotne number", 4);
            telemetry.update();
            /*
            driveSystem.vertical(13.5, 0.7);
            grabSkyStone(5);
            arm.openClaw();
            driveSystem.turn(5, 0.3);
            moveAndGrabStone(4);
            driveSystem.vertical(-6, 0.5);
            arm.openClaw();
            driveSystem.turn(5, 0.3);
            driveSystem.lateral(-4, 0.5);
            moveAndGrabStone(6);
            */

            //Brian's version of the code
            driveSystem.vertical(13.5, 0.7);
            placeSkystoneInScoredFoundation(5);
            driveSystem.vertical(-68, 0.5);
            driveSystem.turn(90, 0.5);
            driveSystem.lateral(8, 0.5);
            placeSkystoneInScoredFoundation(4);
            driveSystem.vertical(-30, 0.5);
        }
    }


    /**
     * Opens the claw and lowers the arm for starting position.
     */
    private void initArm() {
        arm.openClaw();
        arm.lower(0.5);
    }

    /**
     * Approaches the skystone and records its position. Returns 4 if the skystones are in the first and fourth
     * slots. Returns 5 if the skystones are in the second and fifth slots. Returns 6 if the skystones
     * are in the third and sixth slots.
     */
    private int locateSkystone456() {
        driveSystem.vertical(21, 0.5);
        driveSystem.lateral(3.25, 0.5);
        if (seesSkystone()) {
            driveSystem.lateral(2, 0.5);
            driveSystem.vertical(13.5, 0.7);
            return 6;
        }
        driveSystem.lateral(7, 0.3);
        sleep(500);
        if (seesSkystone()) {
            driveSystem.lateral(1.5, 0.5);
            driveSystem.vertical(13.5, 0.7);
            return 5;
        }
        return 4;
    }

    /**
     * Returns true if the skystone is in the center of the camera's field of view.
     */
    private boolean seesSkystone() {
        List<Recognition> recognitions = vision.getRecognitions();
        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals(TTVision.LABEL_SKYSTONE)) {
                Vector2 center = TTVision.getCenter(recognition);
                if (SKYSTONE_BOUNDING_BOX.contains(center)) {
                    return true;
                }
            }
        }
        return false;
    }

    /*Starts from the starting pos and moves grab the block
      at that specific block pos then faces the foundation
     */
    private void grabSkyStone(int stoneNum) {
        arm.closeClaw();
        sleep(750);
        arm.liftTimed(0.25, 0.5);
        sleep(500);
        driveSystem.vertical(-10 + stoneNum, 0.7);
        driveSystem.lateral(-80 + stoneNum * 8, 0.5);
    }
    private void placeSkystoneInScoredFoundation(int stoneNum){
        arm.closeClaw();
        sleep(750);
        arm.liftTimed(0.25, 0.5);
        sleep(500);
        driveSystem.vertical(-12, 0.5);
        driveSystem.turn(-90, 0.5);
        driveSystem.vertical(100 - stoneNum * 8 , 0.5);
        arm.liftTimed(1, 0.5);
        arm.openClaw();
        //6, 60
        //5, 68
        //4, 76
    }

    private void moveAndGrabStone(int stoneNum){
        driveSystem.lateral(-stoneNum * 8 + 80, 0.5);
        driveSystem.vertical(15.5 + stoneNum, 0.7);
        arm.lower(0.5);
        sleep(750);
        grabSkyStone(stoneNum);
    }
    @Override
    protected void onStop() {
    }


}

