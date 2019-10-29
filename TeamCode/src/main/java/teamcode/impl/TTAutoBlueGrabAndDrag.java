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

@Autonomous(name = "TT Auto Blue Grab And Drag")
public class TTAutoBlueGrabAndDrag extends TTOpMode {

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
        skystonePos = locateSkystone();
        if(skystonePos == 6) {
            grabSkyStone(6);
        } else if(skystonePos == 5){
            grabSkyStone(5);
        } else {
            grabSkyStone(4);
        }
        driveSystem.brake();
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
    private int locateSkystone() {
        driveSystem.vertical(20, 0.5);
        driveSystem.lateral(2, 0.5);
        if (seesSkystone()) {
            driveSystem.lateral(2, 0.5);
            return 6;
        }
        driveSystem.lateral(8, 0.3);
        sleep(1000);
        if (seesSkystone()) {
            driveSystem.lateral(1.5, 0.5);
            return 5;
        }
        driveSystem.lateral(9.5, 0.3);
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
        driveSystem.vertical(14.5, 0.7);
        arm.closeClaw();
        sleep(750);
        arm.liftTimed(0.25, 0.5);
        sleep(500);
        driveSystem.vertical(-27.5, 0.7);
        driveSystem.turn(-88, 0.25);
        moveToFoundation(stoneNum);
        pullFoundation();
        driveSystem.brake();
    }

    //Moves towards the foundation and turns to face it
    private void moveToFoundation(int stoneNum) {
        driveSystem.vertical(120.5 - stoneNum * 8, 0.7);
        driveSystem.turn(88, 0.7);
        arm.liftTimed(1, 0.5);
        driveSystem.vertical(32, 0.6);
        sleep(250);
        arm.openClaw();
    }

    private void pullFoundation() {
        driveSystem.lateral(-4.5, 0.7);
        driveSystem.vertical(2, 0.7);
        arm.lower(0.5);
        sleep(250);
        driveSystem.vertical(-60.5, 0.5);
        arm.liftTimed(1, 0.5);
        sleep(250);
        arm.closeClaw();
        driveSystem.lateral(41.5, 0.7);
        arm.lower(0.5);
    }

    @Override
    protected void onStop() {
    }


}

