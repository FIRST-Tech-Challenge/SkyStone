package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base_classes.AutoBot;

/**
 * This is a basic opmode.
 * PLEASE USE GETTERS AND SETTERS INSTEAD OF DIRECTLY SETTING VARIABLES
 * <p>
 * TODO: make robot able to deal with the fact that there is another robot on the same team
 * TODO: make robot carry block
 */

@Autonomous(name = "Basic Auto", group = "Auto")
public class BaseAuto extends LinearOpMode {

    public AutoBot robot = new AutoBot(this);
    public ElapsedTime runtime = new ElapsedTime();

    /**
     * stage of auto: 0 is initial, 1 is strafing to position to sense blocks, 2 is strafing to
     * front of sky stone, 3 is approaching sky stone, 4 is backing up from sky stone,
     */
    public int stageNum = 0;

    public float targetX;
    public float targetY;
    public float stoneXTolerance = 3; // how close (x-wise) (left/right) the robot needs to be in order to pick up a stone
    public float stoneYTolerance = 2; // how close (y-wise) the edge of the robot needs to be to the edge of a stone in order to pick up a stone
    public float safeDistance = 9; // how far the robot needs to go back before strafing back
    public float dropPosition = 12; // what x position the robot can drop the sky stone at

    public float getSafeDistance() {
        return safeDistance;
    }

    public void setSafeDistance(float safeDistance) {
        this.safeDistance = safeDistance;
    }

    public float getDropPosition() {
        return dropPosition;
    }

    public void setDropPosition(float dropPosition) {
        this.dropPosition = dropPosition;
    }

    public float getStoneXTolerance() {
        return stoneXTolerance;
    }

    public void setStoneXTolerance(float stoneXTolerance) {
        this.stoneXTolerance = stoneXTolerance;
    }

    public float getStoneYTolerance() {
        return stoneYTolerance;
    }

    public void setStoneYTolerance(float stoneYTolerance) {
        this.stoneYTolerance = stoneYTolerance;
    }

    public float getTargetX() {
        return targetX;
    }

    public void setTargetX(float targetX) {
        this.targetX = targetX;
    }

    public float getTargetY() {
        return targetY;
    }

    public void setTargetY(float targetY) {
        this.targetY = targetY;
    }

    public int getStageNum() {
        return stageNum;
    }

    public void setStageNum(int stageNum) {
        this.stageNum = stageNum;
    }

    public AutoBot getRobot() {
        return robot;
    }

    public void setRobot(AutoBot robot) {
        this.robot = robot;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init();
        robot.initTracking();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of auto (driver presses STOP)
        while (opModeIsActive()) {

            robot.getNav().updateView();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            float robotX = this.getRobot().getNav().getRobotX();
            float robotY = this.getRobot().getNav().getRobotY();
            float alliance = this.getRobot().getNav().getAlliance();

            switch (this.getStageNum()) {
                case 0: // if just starting up
                    if (robotY > 0) {
                        this.getRobot().strafeRight(1);
                        this.getRobot().getNav().setAlliance(1);
                        this.setStageNum(1);
                    } else if (robotY < 0) {
                        this.getRobot().strafeLeft(1);
                        this.getRobot().getNav().setAlliance(-1);
                        this.setStageNum(1);
                    }
                case 1:
                    if (this.getRobot().getNav().getSkyStonePosition(0) != -1 && this.getRobot().getNav().getSkyStonePosition(1) != -1) {
                        this.setTargetX(Math.min(this.getRobot().getNav().getSkyStoneCenterX(0), this.getRobot().getNav().getSkyStoneCenterX(1)));
                        this.setTargetY(this.getRobot().getNav().getSkyStoneCenterY(0));
                        this.setStageNum(2);
                    }
                case 2:
                    if (Math.abs(robotX - this.getTargetX()) <= this.getStoneXTolerance()) {
                        this.setStageNum(3);
                        this.getRobot().driveStraight(1);
                    } else {
                        if (robotX > this.getTargetX() && this.getRobot().getLeftPower() * alliance > 0) {
                            this.getRobot().strafeRight(alliance);
                        } else if (robotX < this.getTargetX() && this.getRobot().getLeftPower() * alliance < 0) {
                            this.getRobot().strafeLeft(alliance);
                        }
                    }
                case 3:
                    if (robotY * alliance - alliance * this.getRobot().getRobotLength() / 2 - 24 <= this.getStoneYTolerance()) {
                        this.getRobot().stopDriving();
                        // TODO: grab stone
                        this.setStageNum(4);
                        this.getRobot().driveStraight(-1);
                    }
                case 4:
                    if (robotY * alliance >= this.getSafeDistance() + 24) {
                        this.getRobot().strafeLeft(alliance);
                        this.setStageNum(5);
                    }
                case 5:
                    if (robotX >= this.getDropPosition()) {
                        this.getRobot().stopDriving();
                        // TODO: release stone
                        this.setStageNum(6);
                        // done
                    }
            }
        }
    }
}
