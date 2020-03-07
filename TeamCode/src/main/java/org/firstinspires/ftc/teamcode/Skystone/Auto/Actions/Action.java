package org.firstinspires.ftc.teamcode.Skystone.Auto.Actions;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.*;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import java.util.ArrayList;

public class Action {
    Robot robot;

    private ActionType actionType;
    private Point actionPoint;
    private boolean isExecuteOnEndOfPath;
    ActionState actionState = ActionState.PENDING;
    int liftHeight;
    private long actionsStartTime;

    ArrayList<MotionAction> motionActions;

    public Action(ActionType type, Point executePoint, Robot robot) {
        motionActions = new ArrayList<>();
        this.robot = robot;

        this.actionType = type;
        this.actionPoint = executePoint;

        this.isExecuteOnEndOfPath = false;

        generateActions();
    }

    public Action(ActionType type, Point executePoint, Robot robot, int liftHeight) {
        motionActions = new ArrayList<>();

        this.robot = robot;
        this.liftHeight = liftHeight;
        this.actionType = type;
        this.actionPoint = executePoint;

        this.isExecuteOnEndOfPath = false;

        generateActions();
    }

    public Action(ActionType type, Robot robot, boolean isExecuteOnEndOfPath, int liftHeight) {
        motionActions = new ArrayList<>();

        this.robot = robot;
        this.liftHeight = liftHeight;

        this.actionType = type;

        this.isExecuteOnEndOfPath = isExecuteOnEndOfPath;

        generateActions();
    }

    public Action(ActionType type, Robot robot, boolean isExecuteOnEndOfPath) {
        motionActions = new ArrayList<>();

        this.robot = robot;

        this.actionType = type;

        this.isExecuteOnEndOfPath = isExecuteOnEndOfPath;

        generateActions();
    }

    public void executeAction(long currentTime) {
        if (actionState == ActionState.COMPLETE) {
            return;
        }

        if(actionState == ActionState.PENDING) {
            this.actionsStartTime = currentTime;
            actionState = ActionState.PROCESSING;
        }

        boolean hasPendingActions = false;

        long timeSinceActionStart = currentTime - actionsStartTime;
        for (int i = 0; i < motionActions.size(); i++) {
            MotionAction motionAction = motionActions.get(i);
            if(timeSinceActionStart >= motionAction.getDelayStartTime()) {
                motionAction.executeMotion();
            }
            if(motionAction.getStatus() != ActionState.COMPLETE) {
                hasPendingActions = true;
            }
        }

        if (!hasPendingActions) {
            actionState = ActionState.COMPLETE;
        }
    }

    private void generateActions() {
        if (actionType == ActionType.EXTEND_OUTTAKE) {
            generateExtendOuttakeActions();
        } else if (actionType == ActionType.DROPSTONE_AND_RETRACT_OUTTAKE) {
            generateDropStoneAndRetractOuttakeActions();
        } else if (actionType == ActionType.EXTEND_FOUNDATION) {
            generateExtendFoundationActions();
        } else if (actionType == ActionType.RELEASE_FOUNDATION) {
            generateReleaseFoundationActions();
        } else if (actionType == ActionType.START_INTAKE) {
            generateStartIntakeActions();
        } else if (actionType == ActionType.STOP_INTAKE) {
            generateStopIntakeActions();
        }
    }

    private void generateExtendOuttakeActions() {
        motionActions.add(new MotionAction(robot.getIntakeLeft(), 0, 0, robot));
        motionActions.add(new MotionAction(robot.getIntakeRight(), 0, 0, robot));

        motionActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_PUSHED, 0, robot));
        motionActions.add(new MotionAction(robot.getBackClamp(), robot.BACKCLAMP_CLAMPED, 0, robot));
        motionActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_RETRACTED, 550, robot));

        motionActions.add(new MotionAction(robot.getBackClamp(), robot.BACKCLAMP_CLAMPED, 800, robot));
        motionActions.add(new MotionAction(robot.getFrontClamp(), robot.FRONTCLAMP_CLAMPED, 800, robot));
        motionActions.add(new MotionAction(robot.getOuttakeSpool(), 1, liftHeight, 1300, robot));

        motionActions.add(new MotionAction(robot.getOuttakeExtender(), robot.OUTTAKE_SLIDE_EXTENDED, 1500, robot));
    }

    private void generateDropStoneAndRetractOuttakeActions() {
        motionActions.add(new MotionAction(robot.getBackClamp(), robot.BACKCLAMP_RELEASED, 0, robot));
        motionActions.add(new MotionAction(robot.getFrontClamp(), robot.FRONTCLAMP_RELEASED, 0, robot));

        motionActions.add(new MotionAction(robot.getOuttakeExtender(), robot.OUTTAKE_SLIDE_RETRACTED, 250, robot));

        motionActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_RETRACTED, 0, robot));

        motionActions.add(new MotionAction(robot.getOuttakeSpool(), 0, 0, 250, robot));
        motionActions.add(new MotionAction(robot.getOuttakeSpool(), 0, 0, 1500, robot));
        motionActions.add(new MotionAction(robot.getOuttakeExtender(), robot.OUTTAKE_SLIDE_RETRACTED, 250, robot));


    }

    private void generateExtendFoundationActions() {
        motionActions.add(new MotionAction(robot.getLeftFoundation(), robot.LEFTFOUNDATION_EXTENDED, 0, robot));
        motionActions.add(new MotionAction(robot.getRightFoundation(), robot.RIGHTFOUNDATION_EXTENDED, 0, robot));
    }

    private void generateReleaseFoundationActions() {
        motionActions.add(new MotionAction(robot.getLeftFoundation(), robot.LEFTFOUNDATION_RETRACTED, 0, robot));
        motionActions.add(new MotionAction(robot.getRightFoundation(), robot.RIGHTFOUNDATION_RETRACTED, 0, robot));
    }

    private void generateStartIntakeActions() {
        motionActions.add(new MotionAction(robot.getOuttakeSpool(), 0, liftHeight, 0, robot));

        motionActions.add(new MotionAction(robot.getBackClamp(), robot.BACKCLAMP_CLAMPED, 0, robot));
        motionActions.add(new MotionAction(robot.getFrontClamp(), robot.FRONTCLAMP_RELEASED, 0, robot));

        motionActions.add(new MotionAction(robot.getIntakeLeft(), 1, 0, robot));
        motionActions.add(new MotionAction(robot.getIntakeRight(), 1, 0, robot));
    }

    private void generateStopIntakeActions() {
        motionActions.add(new MotionAction(robot.getOuttakeSpool(), 0, liftHeight, 0, robot));

        motionActions.add(new MotionAction(robot.getIntakeLeft(), -1, 0, robot));
        motionActions.add(new MotionAction(robot.getIntakeRight(), -1, 0, robot));
        motionActions.add(new MotionAction(robot.getIntakeLeft(), 1, 500, robot));
        motionActions.add(new MotionAction(robot.getIntakeRight(), 1, 500, robot));
        motionActions.add(new MotionAction(robot.getIntakeLeft(), 0, 1000, robot));
        motionActions.add(new MotionAction(robot.getIntakeRight(), 0, 1000, robot));
    }

    public Point getActionPoint() {
        return actionPoint;
    }

    public ActionState getActionState() {
        return actionState;
    }

    public boolean isExecuteOnEndOfPath() {
        return isExecuteOnEndOfPath;
    }

    public ActionType getActionType() {
        return actionType;
    }
}
