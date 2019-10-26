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

@Autonomous(name = "TT Auto Blue")
public class TTAutoBlue extends TTOpMode {

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
        locateSkystone();
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
    private void locateSkystone() {
        driveSystem.lateral(3.5, 0.5);
        driveSystem.vertical(20, 0.5);
        if (seesSkystone()) {
            grabSkyStone(6);
        }
        driveSystem.lateral(7, 0.3);
        sleep(1000);
        if (seesSkystone()) {
            grabSkyStone(5);
        }
        driveSystem.lateral(8, 0.3);
        grabSkyStone(4);
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
        driveSystem.vertical(12.5, 0.7);
        arm.closeClaw();
        sleep(500);
        driveSystem.vertical(-19, 0.7);
        driveSystem.turn(-90, 0.25);
        moveToFoundation(stoneNum);
        pullFoundation();
    }

    //Moves towards the foundation and turns to face it
    private void moveToFoundation(int stoneNum) {
        driveSystem.vertical(120.5 - stoneNum * 8, 0.7);
        driveSystem.turn(90, 0.7);
        arm.liftTimed(1, 0.5);
        driveSystem.vertical(13 + stoneNum * 3, 0.6);
        sleep(250);
        arm.openClaw();
    }

    private void pullFoundation() {
        driveSystem.lateral(-6.5, 0.7);
        driveSystem.vertical(2, 0.7);
        arm.lower(0.5);
        sleep(250);
        driveSystem.vertical(-53, 0.5);
        arm.liftTimed(1, 0.5);
        driveSystem.lateral(46.5, 0.7);
        arm.lower(0.5);

    }

    @Override
    protected void onStop() {
    }


}

