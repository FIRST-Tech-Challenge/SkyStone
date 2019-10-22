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

@Autonomous(name = "TT Auto Red")
public class TTAutoRed extends TTOpMode {

    private static final BoundingBox2D SKYSTONE_POS_1 = new BoundingBox2D(0, 0, 0, 0);
    private static final BoundingBox2D SKYSTONE_POS_2 = new BoundingBox2D(0, 0, 0, 0);
    private static final BoundingBox2D SKYSTONE_POS_3 = new BoundingBox2D(0, 0, 0, 0);

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
        setArmStartPos();
        grabSkyStone(5);
//        skystonePos = scanStones();
//        telemetry = TTOpMode.currentOpMode().telemetry;
//        telemetry.addData("Stone Found", skystonePos + 3);
//        telemetry.update();
//        if(skystonePos == 1){
//            grabSkyStone(4);
//        } else if(skystonePos == 2){
//            grabSkyStone(5);
//        } else if (skystonePos == 3){
//            grabSkyStone(6);
//        }
    }

    @Override
    protected void onStop() {
    }

    /**
     * Returns the position of the skystones. Returns 1 if the stones are in the first and fourth
     * slots. Returns 2 if the stones are in the second and fifth slots. Returns 3 if the stones
     * are in the third and sixth slots.
     */
    private int scanStones() {
        List<Recognition> recognitions = vision.getRecognitions();
        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals(TTVision.LABEL_SKYSTONE)) {
                Vector2 center = TTVision.getCenter(recognition);
                if (SKYSTONE_POS_1.contains(center)) {
                    return 3;
                } else if (SKYSTONE_POS_2.contains(center)) {
                    return 2;
                } else if (SKYSTONE_POS_3.contains(center)) {
                    return 1;
                }
            }
        }
        return 1; // assume left position if image recognition fails.
    }

    //Opens the claw and lowers the arm for starting pos
    private void setArmStartPos() {
        arm.openClaw();
        arm.lower(0.5);
    }

    /*Starts from the starting pos and moves grab the block
      at that specific block pos then faces the foundation
     */
    private void grabSkyStone(int stoneNum) {
        driveSystem.lateral(-(41.5 - stoneNum * 8), 0.3);
        driveSystem.vertical(31.5, 0.5);
        arm.closeClaw();
        sleep(500);
        driveSystem.vertical(-15, 0.5);
        driveSystem.turn(90, 0.5);
        moveToFoundation(stoneNum);
        pullFoundation();
    }

    //Moves towards the foundation and turns to face it
    private void moveToFoundation(int stoneNum) {
        driveSystem.vertical(120 - stoneNum * 8, 0.5);
        sleep(250);
        driveSystem.turn(-90, 0.5);
        arm.liftTimed(1, 0.5);
        sleep(250);
        driveSystem.vertical(20, 0.5);
        sleep(500);
        arm.openClaw();
    }

    private void pullFoundation() {
        driveSystem.lateral(4, 0.5);
        driveSystem.vertical(2, 0.5);
        arm.lower(0.5);
        sleep(500);
        driveSystem.vertical(-45, 0.5);
        arm.liftTimed(1, 0.5);
        driveSystem.lateral(-44, 0.5);

    }
}