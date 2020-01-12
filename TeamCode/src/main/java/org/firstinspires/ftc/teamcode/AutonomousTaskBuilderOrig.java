package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class AutonomousTaskBuilderOrig {
    int delay;
    int SkyStonePosition = -1;
    String startingPositionModes;
    String parking;
    String deliverRoute;
    String moveFoundation;
    String isParkOnly;
    String isTwoSkystones;

    RobotProfile robotProfile;
    RobotHardware robotHardware;
    RobotNavigator navigator;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    SequentialComboTask pickUpTask;
    ParallelComboTask toPickUpTask;
    ParallelComboTask fromPickUpTask;
    SequentialComboTask dropOffTask;
    RobotPosition lastPos;
    int stoneX, dropY, bridgeY, parkY, parkWall, parkNeutral;

    public AutonomousTaskBuilderOrig(DriverOptions driverOptions, int skyStonePosition, RobotHardware robotHardware, RobotNavigator navigator, RobotProfile robotProfile){
        this.delay = driverOptions.getDelay();
        this.SkyStonePosition = skyStonePosition;
        this.startingPositionModes = driverOptions.getStartingPositionModes();
        this.parking = driverOptions.getParking();
        this.deliverRoute = driverOptions.getDeliverRoute();
        this.moveFoundation = driverOptions.getMoveFoundation();
        this.isParkOnly = driverOptions.getIsParkOnly();
        this.isTwoSkystones = driverOptions.getIsTwoSkystones();
        this.robotHardware = robotHardware;
        this.navigator = navigator;
        this.robotProfile = robotProfile;
        setupPickUpTask();
        setupToPickUpTask();
        setupFromPickUpTask();

    }

    void addMovement(RobotPosition beginP, RobotPosition endP) {
        PIDMecanumMoveTask task = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        task.setPath(beginP, endP);
        taskList.add(task);
        lastPos = endP;
    }

    //for from starting point to parking point only
    public ArrayList<RobotControl> buildParkingOnlyTask(String parking){
        if(this.delay > 0){
            taskList.add(new RobotSleep(delay * 1000 ));
        }
        Logger.logFile("in buildTask for parking only: "+ parking);
            if(isParkOnly.equals("yes")){
                switch (startingPositionModes){
                    case "RED_2":
                        if(parking.equals("BridgeNeutral")){
                            addMovement(new RobotPosition(0, 0 , 0), new RobotPosition(65, 0, 0));
                            addMovement(lastPos, new RobotPosition(65, -88, 0));
                        } else if (parking.equals("BridgeWall")){
                            addMovement(lastPos, new RobotPosition(0, -88, 0));
                        }
                        break;
                    case "RED_3":
                        if(parking.equals("BridgeNeutral")){
                            addMovement(new RobotPosition(0, 0 , 0), new RobotPosition(68, -28, 0));
                            //addMovement(lastPos, new RobotPosition(58, -43, 0));
                        } else if (parking.equals("BridgeWall")){
                            addMovement(new RobotPosition(0, 0 , 0), new RobotPosition(0, -30, 0));
                        }
                        break;
                    case "BLUE_2":
                        break;
                    case "BLUE_3":
                        break;
                }
            }

        return taskList;
    }

    public ArrayList<RobotControl> buildDeliverOneStoneTask(){
        if(this.delay > 0){
            taskList.add(new RobotSleep(delay * 1000 ));
        }

        stoneX = 70;
        switch (startingPositionModes){
            case ("RED_2") :
                dropY = -210;
                parkY = -85;
                parkWall = 10;
                parkNeutral = 60;
                Logger.logFile("Red 2");
                goToSkyStone();
                break;
            case("RED_3"):
                dropY = -150;
                parkY = -20;
                parkWall = 10;
                parkNeutral = 60;
                // move up toward stone row
               goToSkyStone();
                // do pick up now
                break;
            case("BLUE_2"):
                dropY = 210;
                parkY = 85;
                parkWall = 50;
                parkNeutral = 120;
                goToSkyStone();
                break;
            case("BLUE_3"):
                dropY = 150;
                parkY = 20;
                parkWall = 50;
                parkNeutral = 120;
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(stoneX, 0, 0));
                goToSkyStone();
        }
        Logger.logFile("before pick up task");
        Logger.logFile("size of pick up task" + toPickUpTask.taskList.size() + fromPickUpTask.taskList.size());
        //setupPickUpTask();
        taskList.add(toPickUpTask);
//        taskList.add(new RobotSleep(500));
        taskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase, 500));
        taskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE));
        taskList.add(fromPickUpTask);
        taskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftOrigPos,500));
//
//        Logger.logFile("done");
//        taskList.add(new RobotSleep(500));
//        Logger.logFile("last x:" + lastPos.getX());
//        Logger.logFile("last y:" + lastPos.getY());
//        Logger.logFile("going to x:" + deliverRoute(deliverRoute));
//        Logger.logFile("going to y:" + lastPos.getY());
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), lastPos.getY(), 0)); // was stoneX/2
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), dropY, 0));
        setupDropOffTask();
        taskList.add(dropOffTask);
        addMovement(lastPos, new RobotPosition(stoneX, dropY, 0));
        taskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftOrigPos, 0));
//        //new
//        //need to rotate robot
        taskList.add(new HookPositionTask(robotHardware, robotProfile, RobotHardware.HookPosition.HOOK_ON));
        parking();

        return taskList;
    }

    public ArrayList<RobotControl> buildDropTwoStoneTask(){
        stoneX = 72;
        switch (startingPositionModes){
            case ("RED_2") :
                bridgeY = -135;
                parkY = -85;
                parkWall = 10;
                parkNeutral = 60;
                goToSkyStone();
                break;
                case("BLUE_2"):
                Logger.logFile("ppp");
                bridgeY = 130;
                parkY = 90;
                parkWall = 5;
                parkNeutral = 60;
                goToSkyStone();
                break;
//
        }

        taskList.add(pickUpTask);
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), lastPos.getY(), 0));
        setupDropOffTask();
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), bridgeY, 0));
        taskList.add(dropOffTask);
        addMovement(lastPos, new RobotPosition(lastPos.getX(), goToSkyStone2(SkyStonePosition), 0));
        addMovement(lastPos, new RobotPosition(lastPos.getX(), goToSkyStone2(SkyStonePosition), 0));
        addMovement(lastPos, new RobotPosition(stoneX, lastPos.getY(), 0));
        taskList.add(pickUpTask);
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), lastPos.getY(), 0));
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), bridgeY, 0));
        taskList.add(dropOffTask);
        parking();

        return taskList;
    }

    void goToSkyStone() { //this method is where the robot goes to position itself before pickup at the beginning of autonomous
        Logger.logFile("startingPositionModes == " + startingPositionModes);
        Logger.logFile("skystoneposition == " + SkyStonePosition);
        if (startingPositionModes.equals("RED_2")){
            if (SkyStonePosition == 0) {
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(20, 70, 0));
                addMovement(lastPos, new RobotPosition(stoneX, 70, 0));
            } else if (SkyStonePosition == 1) {
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, 49,0));
                addMovement(lastPos, new RobotPosition(stoneX, 49, 0));
            } else if (SkyStonePosition == 2) {
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, 30, 0));
                addMovement(lastPos, new RobotPosition(stoneX, 30, 0));
            }
        } else if (startingPositionModes.equals("RED_3")){
            if (SkyStonePosition == 0) {
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(stoneX, 60, 0));
            }
            else if (SkyStonePosition == 1) {
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(stoneX, 40, 0));
            }
            else if (SkyStonePosition == 2) {
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(stoneX, 20, 0));
            }
        } else if (startingPositionModes.equals("BLUE_2")) {
            Logger.logFile("hee");
            if (SkyStonePosition == 0) {  //special case, cannot get sky stone, grab anyone, assume grab the closest one and move on
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, -33, 0));
                addMovement(lastPos, new RobotPosition(stoneX, -33, 0));
            }
            else if (SkyStonePosition == 1) {
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, -33, 0));
                addMovement(lastPos, new RobotPosition(stoneX, -33, 0));
            }
            else if (SkyStonePosition == 2) {
                addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, -26, 0));
                addMovement(lastPos, new RobotPosition(stoneX, -26, 0));
                Logger.logFile("wee");
            }
        } else if (startingPositionModes.equals("BLUE_3")) {
            if (SkyStonePosition == 0) {
                addMovement(lastPos, new RobotPosition(stoneX, -53, 0));
            } else if (SkyStonePosition == 1) {
                addMovement(lastPos, new RobotPosition(stoneX, -33, 0));
            } else if (SkyStonePosition == 2) {
                addMovement(lastPos, new RobotPosition(stoneX, -13, 0));
            }
        }

                // do pick up now
    }

    double goToSkyStone2(double y){
        switch(SkyStonePosition){
            case(0):
                if(startingPositionModes.equals("RED_2")){y = 60;}
                else if(startingPositionModes.equals("BLUE_2")){y = -9;}
                break;
            case(1):
                if(startingPositionModes.equals("RED_2")){y = 40;}
                else if(startingPositionModes.equals("BLUE_2")){y = 17;}
                break;
            case(2):
                if(startingPositionModes.equals("RED_2")){y = 20;}
                else if(startingPositionModes.equals("BLUE_2")){y = 35;}
                break;
        }
        return y;
    }

    int deliverRoute(String deliverRoute){
        this.deliverRoute = deliverRoute;
        if(deliverRoute.equals("BridgeWall")){
            return parkWall;
        } else{
            return parkNeutral;
        }
    }

    void setupPickUpTask() {
        ArrayList<RobotControl> pickUpTaskList = new ArrayList<RobotControl>();

        //lift up
        pickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 500));
        pickUpTaskList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
        //slide out
        pickUpTaskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPosAutonomous, 500));
        //lift down
        pickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase, 500));
        pickUpTaskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE)); //this is currently close
        pickUpTaskList.add(new RobotSleep(500));
        //lift up
        pickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftGrabExtra*4 +
                robotProfile.hardwareSpec.liftStoneBase, 500));
        //slide in
        pickUpTaskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 500));
        pickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftOrigPos,500));

        pickUpTask = new SequentialComboTask();
        pickUpTask.setTaskList(pickUpTaskList);
    }

    // parallel task before picking up block
    void setupToPickUpTask() {
        ArrayList<RobotControl> toPickUpTaskList = new ArrayList<RobotControl>();

        //lift up
        toPickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 500));
        Logger.logFile("Lift Height 1: " + robotHardware.liftMotor.getCurrentPosition());

        toPickUpTaskList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
        //slide out
        toPickUpTaskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPosAutonomous, 500));
        //toPickUpTaskList.add(new RobotSleep(2000));
        //lift down
        //toPickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase, 500));
//        toPickUpTaskList.add(new RobotSleep(500));
        //toPickUpTaskList.add(new ClampOpenCloseTask(robotHardware,robotProfile,RobotHardware.ClampPosition.CLOSE));
        toPickUpTask = new ParallelComboTask();
        toPickUpTask.setTaskList(toPickUpTaskList);
    }


    // parallel task after picking up block
    void setupFromPickUpTask() {
        ArrayList<RobotControl> fromPickUpTaskList = new ArrayList<RobotControl>();

        //lift up
        fromPickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftGrabExtra*4 +
                robotProfile.hardwareSpec.liftStoneBase, 500));
        //slide in
        fromPickUpTaskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 500));

        fromPickUpTask = new ParallelComboTask();
        fromPickUpTask.setTaskList(fromPickUpTaskList);
    }

    void setupDropOffTask(){
        ArrayList <RobotControl> dropOffTaskList = new ArrayList<RobotControl>();
        dropOffTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftPerStone*2, 500));
        addMovement(lastPos, new RobotPosition(stoneX, dropY, 0));
        dropOffTaskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPosAutonomous, 500));
        dropOffTaskList.add(new SetLiftPositionTask(robotHardware,robotProfile,robotProfile.hardwareSpec.liftPerStone,500));
        dropOffTaskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
        dropOffTaskList.add(new SetLiftPositionTask(robotHardware,robotProfile,robotProfile.hardwareSpec.liftPerStone*2,500));
        dropOffTaskList.add(new SetSliderPositionTask(robotHardware,robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 500));
        dropOffTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftOrigPos,500));
        dropOffTask = new SequentialComboTask();
        dropOffTask.setTaskList(dropOffTaskList);
    }

    void addMovementAndSlide(RobotPosition beginP, RobotPosition endP) {
        ArrayList<RobotControl> parallelList = new ArrayList<RobotControl>();
        ArrayList<RobotControl> sequentialList = new ArrayList<RobotControl>();

//       sequentialList.add(new RobotSleep(100));
        sequentialList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 500));
//       combo
//      List2.add(new RobotSleep(100));
        sequentialList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPosAutonomous, 500));
        sequentialList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
        SequentialComboTask deliverPosition = new SequentialComboTask();
        deliverPosition.setTaskList(sequentialList);
        PIDMecanumMoveTask moveTask = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        moveTask.setPath(beginP, endP);

        ParallelComboTask moveAndDeliver = new ParallelComboTask();
        parallelList.add(deliverPosition);
        parallelList.add(moveTask);
        moveAndDeliver.setTaskList(parallelList);
        taskList.add(moveAndDeliver);

        lastPos = endP;
    }

    public void parking(){
        if(parking.equals("BridgeWall")) {
            addMovement(lastPos, new RobotPosition(parkWall, parkY, 0));
        } else if (parking.equals("BridgeNeutral")){
            addMovement(lastPos, new RobotPosition(parkNeutral, parkY, 0));
        }
    }
}
