package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class AutonomousTaskBuilder {
    int SkyStonePosition=-1;
    String startingPositionModes;
    RobotProfile robotProfile;
    RobotHardware robotHardware;
    RobotNavigator navigator;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    SequentialComboTask pickUpTask;
    double zero = 0;
    RobotPosition lastPos;
    int stoneX, dropY, parkY;

    public AutonomousTaskBuilder(int skyStonePosition, String startingPositionModes, RobotHardware robotHardware, RobotNavigator navigator, RobotProfile robotProfile){
        this.SkyStonePosition = skyStonePosition;
        this.startingPositionModes = startingPositionModes;
        this.robotHardware = robotHardware;
        this.navigator = navigator;
        this.robotProfile = robotProfile;
        setupCombos();
    }

    void addMovement(RobotPosition beginP, RobotPosition endP) {
        PIDMecanumMoveTask task = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        task.setPath(beginP, endP);
        taskList.add(task);
        lastPos = endP;
    }

    public ArrayList<RobotControl> buildTask(){
        stoneX = 73;
        switch (startingPositionModes){
            case ("RED_2") :
                dropY = -160;
                parkY = -80;
                if (SkyStonePosition == 0) {
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, 53, 0));
                    addMovement(new RobotPosition(stoneX, 25, zero), new RobotPosition(stoneX, 53, zero));
                }
                else if (SkyStonePosition == 1) {
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, 33, 0));
                    addMovement(new RobotPosition(0, 33, 0), new RobotPosition(stoneX, 33, zero));
                }
                else if (SkyStonePosition == 2) {
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, 13, 0));
                    addMovement(new RobotPosition(0, 13, zero), new RobotPosition(stoneX, 13, zero));
                }
                //add parking the platform if possible
//                addMovement(new RobotPosition(287, 332, 0), new RobotPosition(287,322,0));
//                addMovement(new RobotPosition(287, 322,0), new RobotPosition(287, 332,90));
//                addMovement(new RobotPosition(287,332,90),new RobotPosition(335,332,90));
//                addMovement(new RobotPosition(335,332,90),new RobotPosition(335,175,90));
                break;
            case("RED_3"):
                dropY = -100;
                parkY = -20;
                // move up toward stone row
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(stoneX, 0, 0));
                if (SkyStonePosition == 0) {
                    addMovement(new RobotPosition(stoneX, 0, zero), new RobotPosition(stoneX, 60, zero));
                }
                else if (SkyStonePosition == 1) {
                    addMovement(new RobotPosition(stoneX, 0, zero), new RobotPosition(stoneX, 40, zero));
                }
                else if (SkyStonePosition == 2) {
                    addMovement(new RobotPosition(stoneX, 0, zero), new RobotPosition(stoneX, 20, zero));
                }
                // do pick up now
                break;
            case("BLUE_2"):
                dropY = 160;
                parkY = 80;
                if (SkyStonePosition == 0) {  //special case, cannot get sky stone, grab anyone, assume grab the closest one and move on
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, -33, 0));
                    addMovement(lastPos, new RobotPosition(stoneX, -33, 0));
                }
                else if (SkyStonePosition == 1) {
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, -33, 0));
                    addMovement(lastPos, new RobotPosition(stoneX, -33, 0));
                }
                else if (SkyStonePosition == 2) {
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, -13, 0));
                    addMovement(lastPos, new RobotPosition(stoneX, -53, 0));
                }
                break;
            case("BLUE_3"):
                dropY = 100;
                parkY = 20;
                if (SkyStonePosition == 0) {
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(stoneX, 0, 0));
                    addMovement(new RobotPosition(stoneX, 0, 0), new RobotPosition(stoneX, -53, 0));
                }
                else if (SkyStonePosition == 1) {
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(stoneX, 0, 0));
                    addMovement(new RobotPosition(stoneX, 0, 0), new RobotPosition(stoneX, -33, 0));
                }else if (SkyStonePosition == 2) {
                    addMovement(new RobotPosition(0, 0, 0), new RobotPosition(stoneX, 0, 0));
                    addMovement(new RobotPosition(stoneX, 0, 0), new RobotPosition(stoneX, -13, 0));
                }
        }
        taskList.add(pickUpTask);
        taskList.add(new RobotSleep(1000));     // take a look, selfie time

        addMovement(lastPos, new RobotPosition(stoneX/2, lastPos.getY(), 0));
        addMovement(lastPos, new RobotPosition(stoneX/2, dropY, 0));
        taskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
        addMovement(lastPos, new RobotPosition(stoneX/2, parkY, 0));
        return taskList;
    }


    void setupCombos() {
        ArrayList<RobotControl> comboList = new ArrayList<RobotControl>();
        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 1000));
        comboList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
        comboList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPos, 1000));
        comboList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
                robotProfile.hardwareSpec.liftGrabExtra, 1000));
        comboList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
        comboList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftGrabExtra, 1000));
        pickUpTask = new SequentialComboTask();
        pickUpTask.setTaskList(comboList);
    }
}
