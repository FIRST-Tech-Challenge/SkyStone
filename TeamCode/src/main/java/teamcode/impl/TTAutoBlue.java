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
        skystonePos = scanStones();
        setArmStartPos();
        grabBlock4();
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
                    return 1;
                } else if (SKYSTONE_POS_2.contains(center)) {
                    return 2;
                } else if (SKYSTONE_POS_3.contains(center)) {
                    return 3;
                }
            }
        }
        return 1; // assume left position if image recognition fails.
    }

    public void grabBlock4() {
        driveSystem.lateral(10, 0.25);
        driveSystem.vertical(31.5, 0.25);
        arm.closeClaw();
        sleep(250);
        driveSystem.vertical(-10, 0.25);
        driveSystem.turn(-90, 0.25);
        driveSystem.vertical(83.5, 0.5);
        arm.liftTimed(1, 0.5);
        sleep(250);
        driveSystem.turn(90, 0.25);
        driveSystem.vertical(15.75, 0.25);
        sleep(250);
        arm.lower(0.5);
        driveSystem.turn(90, 0.25);
        driveSystem.lateral(-25, 0.25);
        driveSystem.vertical(4, 0.25);
        arm.openClaw();
        sleep(250);
        arm.liftTimed(1, 0.5);
        driveSystem.vertical(-10, 0.5);

    }

    private void setArmStartPos() {
        arm.openClaw();
        arm.lower(0.5);
    }

}
