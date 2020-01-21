package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class AutonomousTaskBuilder {
    int delay;
    int skyStonePosition = -1;
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
    SequentialComboTask releaseWheelTask;
    ParallelComboTask toPickUpTask;
    ParallelComboTask fromPickUpTask;
    SequentialComboTask dropOffTask;
    RobotPosition lastPos;
    PIDMecanumMoveTask lastMovement;
    SequentialComboTask moveBuilderPlateTask;
    int stoneX, stoneY, dropY, dropX, bridgeY, parkX, parkY, pullPlateY, pullPlateX, finalPushX,
            finalPushY,  parkWall, parkNeutral, afterRotatePlateY;
    double finalHeading;

    public AutonomousTaskBuilder(DriverOptions driverOptions, int skyStonePosition, RobotHardware robotHardware, RobotNavigator navigator, RobotProfile robotProfile) {
        this.delay = driverOptions.getDelay();
        this.skyStonePosition = skyStonePosition;
        this.startingPositionModes = driverOptions.getStartingPositionModes();
        this.parking = driverOptions.getParking();
        this.deliverRoute = driverOptions.getDeliverRoute();
        this.moveFoundation = driverOptions.getMoveFoundation();
        this.isParkOnly = driverOptions.getIsParkOnly();
        this.isTwoSkystones = driverOptions.getIsTwoSkystones();
        this.robotHardware = robotHardware;
        this.navigator = navigator;
        this.robotProfile = robotProfile;
    }

    void addMovement(RobotPosition beginP, RobotPosition endP) {
        lastMovement = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        lastMovement.setPath(beginP, endP);
        taskList.add(lastMovement);
        lastPos = endP;
    }

    //for from starting point to parking point only
    public ArrayList<RobotControl> buildParkingOnlyTask(String parking) {
        if (this.delay > 0) {
            taskList.add(new RobotSleep(delay * 1000 ));
        }

        Logger.logFile("in buildTask for parking only: "+ parking);
            if (isParkOnly.equals("yes")) {
                switch (startingPositionModes) {
                    case "RED_2":
                        parkY = -85;
                        parkWall = 10;
                        parkNeutral = 50;  //60 robot after long distance travel, angle tilted, almost hit the platform, reduce x to avoid crash
//                        if(parking.equals("BridgeNeutral")){
//                            addMovement(new RobotPosition(0, 0 , 0), new RobotPosition(65, 0, 0));
//                            addMovement(lastPos, new RobotPosition(65, -88, 0));
//                        } else if (parking.equals("BridgeWall")){
//                            addMovement(lastPos, new RobotPosition(0, -88, 0));
//                        }
                        break;
                    case "RED_3":
                        parkY = -20;
                        parkWall = 10;
                        parkNeutral = 60;
//                        if(parking.equals("BridgeNeutral")){
//                            addMovement(new RobotPosition(0, 0 , 0), new RobotPosition(68, -28, 0));
//                            //addMovement(lastPos, new RobotPosition(58, -43, 0));
//                        } else if (parking.equals("BridgeWall")){
//                            addMovement(new RobotPosition(0, 0 , 0), new RobotPosition(0, -30, 0));
//                        }
                        break;
                    case "RED_5":
                        parkY = 100;
                        parkWall = 10;
                        parkNeutral = 60;
                                
                    case "BLUE_2":
                        parkY = 85;
                        parkWall = 10; //50;
                        parkNeutral = 60; //120;
                        break;
                    case "BLUE_3":
                        parkY = 40;
                        parkWall = 10;
                        parkNeutral = 60;
                        break;
                    case "BLUE_5":
                        parkY = -80;
                        parkWall = 10;
                        parkNeutral = 60;
                        break;    
                }
                parking();
                //addMovement(new RobotPosition(0, 0, 0), new RobotPosition(deliverRoute(deliverRoute), parkY, 0));
            }
        return taskList;
    }

    // THIS IS THE 29-POINT MOVE
    public ArrayList<RobotControl> buildDeliverOneStoneCompleteTask() {
        if (this.delay > 0) {   // add 1 sleep anyway for debug
            taskList.add(new RobotSleep(delay * 1000 ));
        }

        populateCommonTask();

        ArrayList<RobotPosition> moveToStonePos = new ArrayList<RobotPosition>();
        if (startingPositionModes.endsWith("2")) {
            // RED_2 and BLUE_2 need to move parallel to the wall, then to stone
            moveToStonePos.add(new RobotPosition(0, stoneY, 0));
            moveToStonePos.add(new RobotPosition(stoneX, stoneY, 0));
        } else {
            // RED_3 and BLUE_3 move away from wall to parkNutural, then parallel wall to stone,
            // then move up to stone
            moveToStonePos.add(new RobotPosition(deliverRoute(deliverRoute), 0, 0));
            moveToStonePos.add(new RobotPosition(deliverRoute(deliverRoute), stoneY, 0));
            moveToStonePos.add(new RobotPosition(stoneX, stoneY, 0));
        }

        // move to the stone and lift/extend slide, clamp open
        moveAndSlideToStone(new RobotPosition(0, 0, 0), moveToStonePos);
        //taskList.add(new RobotSleep(5000)); // debug purpose
        // pick up and put it back inside
        taskList.add(pickUpTask);
        //taskList.add(new RobotSleep(15000)); // debug purpose
        moveToPlateAndExtend();
        //taskList.add(dropOffTask);    // move as parallel into moveBuilderPlateTask
        if (moveFoundation.equals("move")) {
            setupDropAndMoveBuilderPlateTaskNew();    // this needs to be here instead of constructor, because we depends on DropX/DropY, etc.
            taskList.add(moveBuilderPlateTask);
        }

        parking();
//
//        // for now, add sleep, and back to original starting position
//        taskList.add(new RobotSleep(5000));
//        addMovement(lastPos, new RobotPosition(5, 0, 0));
        return taskList;
    }

    //dropping two stones can happen in all starting positions except the sky stones are in [0] position.
    //If the sky stones are in [0] position, please see the following options:
    // -- RED_2[0] & BLUE_2[0], option is buildPickUpFirstBlockAndPark(), by selecting the AutonomousOptions' choice of - "first block by wall" & "parking"
    // -- RED_3[0] & BLUE_3[0], option is buildDeliverOneStoneCompleteTask(), which includes pick up one sky stone, remove platform and parking
    //
    //If we choose to deliver two stones across the bridge line and park(25 points), our partner only needs to remove the platform and parking(15 points)
    public ArrayList<RobotControl> buildDropTwoStoneTask() {

        populateCommonTask();

        taskList.add(releaseWheelTask);
        if (startingPositionModes.endsWith("2")) {
            addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, stoneY, 0));
            addMovement(lastPos, new RobotPosition(stoneX, stoneY, 0));
        } else if (startingPositionModes.endsWith("3")) {
            addMovement(new RobotPosition(0, 0, 0), new RobotPosition(deliverRoute(deliverRoute), 0, 0));
            addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), stoneY, 0));
            addMovement(lastPos, new RobotPosition(stoneX, stoneY, 0));
        }

//        if(startingPositionModes.endsWith("2")){
//            addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, stoneY, 0));
//            addMovement(lastPos, new RobotPosition(stoneX, stoneY, 0));
//        }else if(startingPositionModes.endsWith("3")){
//            addMovement(new RobotPosition(0, 0, 0), new RobotPosition(parkNeutral, 0, 0));
//            addMovement(lastPos, new RobotPosition(parkNeutral, stoneY, 0));
//            addMovement(lastPos, new RobotPosition(stoneX, stoneY, 0));
//        }
        taskList.add(pickUpTask);
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), lastPos.getY(), 0));
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), bridgeY, 0));
        taskList.add(dropOffTask);
        addMovement(lastPos, new RobotPosition(lastPos.getX(), goToSkyStone2(skyStonePosition), 0));
        addMovement(lastPos, new RobotPosition(stoneX, lastPos.getY(), 0));
        taskList.add(pickUpTask);
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), lastPos.getY(), 0));
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), bridgeY, 0));
        taskList.add(dropOffTask);
        parking();

        return taskList;
    }

    // TODO: 1/20/2020  return something with position 5's  
    double goToSkyStone2(int skyStonePosition) {
        if (startingPositionModes.equals("RED_2")){ return 26 + (2 - skyStonePosition) *20;}  //40
        else if(startingPositionModes.equals("RED_3")){return 26 + (2 - skyStonePosition) *20;}
        else if(startingPositionModes.equals("BLUE_2")){ return -38 + (skyStonePosition-2) * 20;}
        else if(startingPositionModes.equals("BLUE_3")){ return -80 + skyStonePosition * 20;}
        //return y;
        return 0; //should not be here
    }

    //the robot is either on RED3 or BLUE3, not for RED2 or BLUE2 since possibility crash might be high
    public ArrayList<RobotControl> buildMovePlatformAndParkTask() {
        populateCommonTask();
        taskList.add(releaseWheelTask);
        addMovement(new RobotPosition(0, 0, 0), new RobotPosition(parkWall, 0, 0));
        addMovement(lastPos, new RobotPosition(parkWall, dropY, 0));
        taskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftHomeReadyPos, 500));
        addMovement(lastPos, new RobotPosition(dropX, dropY, 0));
        setupDropAndMoveBuilderPlateTaskNew();
        taskList.add(moveBuilderPlateTask);
        parking();
        return taskList;
    }

    //deliver one stone across the bridge line and park
    public ArrayList<RobotControl> buildDeliverOneStoneOnlyTask() {
        populateCommonTask();

        taskList.add(releaseWheelTask);
        if (startingPositionModes.endsWith("2")) {
            addMovement(new RobotPosition(0, 0, 0), new RobotPosition(0, stoneY, 0));
            addMovement(lastPos, new RobotPosition(stoneX, stoneY, 0));
        } else if (startingPositionModes.endsWith("3")) {
            addMovement(new RobotPosition(0, 0, 0), new RobotPosition(parkNeutral, 0, 0));
            addMovement(lastPos, new RobotPosition(parkNeutral, stoneY, 0));
            addMovement(lastPos, new RobotPosition(stoneX, stoneY, 0));
        }
        taskList.add(pickUpTask);
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), lastPos.getY(), 0));
        addMovement(lastPos, new RobotPosition(deliverRoute(deliverRoute), bridgeY, 0));
        taskList.add(dropOffTask);
        parking();
        return taskList;
    }

    int deliverRoute(String deliverRoute) {
        this.deliverRoute = deliverRoute;

        if (deliverRoute.equals("BridgeWall")) {
            return parkWall;
        } else {
            return parkNeutral;
        }
    }

    void setupPickUpTask() {
        ArrayList<RobotControl> pickUpTaskList = new ArrayList<RobotControl>();
        // measure block distance and slide to position
        //pickUpTaskList.add(new MeasureAndSlideTask(robotHardware, robotProfile, 500));
        //pickUpTaskList.add(new RobotSleep(500));    // give 0.5 second for it to settle down

        // step 1 - lift down
        pickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase, 500));
        pickUpTaskList.add(new RobotSleep(100));    // give 0.5 second for it to settle down
        // step 2 - close the clamp
        pickUpTaskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.CLOSE)); //this is currently close
        // step 3 - lift up
        pickUpTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftGrabExtra +
                robotProfile.hardwareSpec.liftPerStone+ robotProfile.hardwareSpec.liftStoneBase, 500));
        pickUpTask = new SequentialComboTask();
        // step 4 - store stone  and go back to delivery route
        ParallelComboTask slideAndMoveTask = new ParallelComboTask();
        ArrayList<RobotControl> slideAndMoveList = new ArrayList<RobotControl>();
        // parallel task 1 - store the stone, slide back and lift down
        ArrayList<RobotControl> storeStoneList = new ArrayList<RobotControl>();
        //slide in
        storeStoneList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 500));
        // lift down
        storeStoneList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftHomeGrabPos,500));
        SequentialComboTask storeStoneTask = new SequentialComboTask();
        storeStoneTask.setTaskList(storeStoneList);
        // parallel task 2 - move to route
        PIDMecanumMoveTask moveToRoute = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        moveToRoute.setPath(new RobotPosition(stoneX, stoneY, 0), lastPos = new RobotPosition(deliverRoute(deliverRoute), stoneY, 0));
        // compose the parallel task
        slideAndMoveList.add(storeStoneTask);
        slideAndMoveList.add(moveToRoute);
        slideAndMoveTask.setTaskList(slideAndMoveList);
        // add the parallel combo to seq task list
        pickUpTaskList.add(slideAndMoveTask);

        pickUpTask.setTaskList(pickUpTaskList);
        pickUpTask.setTaskName("Pick up stone and move to route");
    }

    void moveToPlateAndExtend() {
//        ArrayList<RobotControl> storeStoneList = new ArrayList<RobotControl>();
//        //slide in
//        storeStoneList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 500));
//        // lift down
//        storeStoneList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftHomeGrabPos,500));
//        SequentialComboTask storeStoneTask = new SequentialComboTask();
//        storeStoneTask.setTaskList(storeStoneList);

        ArrayList<RobotControl> moveToPlateList = new ArrayList<RobotControl>();
//        PIDMecanumMoveTask moveToRoute = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
//        moveToRoute.setPath(new RobotPosition(stoneX, stoneY, 0), lastPos = new RobotPosition(deliverRoute(deliverRoute), stoneY, 0));
//        moveToPlateList.add(moveToRoute);
        // parallel task 1, move across and then move to plate
        PIDMecanumMoveTask moveAcross = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        moveAcross.setPath(lastPos, lastPos = new RobotPosition(deliverRoute(deliverRoute), dropY, 0));
        moveAcross.setPower(0.8);
        moveToPlateList.add(moveAcross);
        PIDMecanumMoveTask moveToPlate = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        moveToPlate.setPath(lastPos, lastPos = new RobotPosition(dropX, dropY, 0));
        moveToPlate.setPower(0.8);
        moveToPlate.setMinPower(0.5);
        moveToPlateList.add(moveToPlate);
        SequentialComboTask moveToPlateTask = new SequentialComboTask();
        moveToPlateTask.setTaskList(moveToPlateList);
        // Parallel task 2, wait for pass under bridge, then lift and extend out
        ArrayList <RobotControl> liftAndExtendList = new ArrayList<RobotControl>();
        liftAndExtendList.add(new WaitForNavigationTask(navigator, new RobotPosition(-10, bridgeY, -1), new RobotPosition(100, dropY, 1)));
        liftAndExtendList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftPerStone*3/2, 500));
        liftAndExtendList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutMax, 500));
        liftAndExtendList.add(new ClampAngleRotationTask(robotHardware, robotProfile, RobotHardware.ClampAnglePosition.SIDE));
        SequentialComboTask liftAndExtendTask = new SequentialComboTask();
        liftAndExtendTask.setTaskList(liftAndExtendList);
        // compose the parallel task
        ParallelComboTask moveAndExtendTask = new ParallelComboTask();
        ArrayList<RobotControl> moveAndExtendList = new ArrayList<RobotControl>();
//        moveAndExtendList.add(storeStoneTask);
        moveAndExtendList.add(liftAndExtendTask);
        moveAndExtendList.add(moveToPlateTask);
        moveAndExtendTask.setTaskList(moveAndExtendList);
        taskList.add(moveAndExtendTask);
    }

    void setupDropOffTask(){
        ArrayList <RobotControl> dropOffTaskList = new ArrayList<RobotControl>();
//        dropOffTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftPerStone*2, 500));
//        addMovement(lastPos, new RobotPosition(stoneX, dropY, 0));
//        dropOffTaskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutMax, 500));
//        dropOffTaskList.add(new SetLiftPositionTask(robotHardware,robotProfile,robotProfile.hardwareSpec.liftPerStone,500));
        dropOffTaskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
        dropOffTaskList.add(new RobotSleep(100));
        dropOffTaskList.add(new ClampAngleRotationTask(robotHardware, robotProfile, RobotHardware.ClampAnglePosition.NORMAL));
        dropOffTaskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.INITIAL));
//        dropOffTaskList.add(new SetLiftPositionTask(robotHardware,robotProfile,robotProfile.hardwareSpec.liftPerStone,500));
        dropOffTaskList.add(new SetSliderPositionTask(robotHardware,robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 100));
        dropOffTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftOrigPos,100));
        dropOffTask = new SequentialComboTask();
        dropOffTask.setTaskList(dropOffTaskList);
        dropOffTask.setTaskName("Drop Off Stone");
    }

    void setupReleaseWheelTask() {
        releaseWheelTask = new SequentialComboTask();
        // do nothing for now
//        releaseWheelTask.addTask(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.RELEASE_1));
//        releaseWheelTask.addTask(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.RELEASE_2));
        releaseWheelTask.addTask(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.INITIAL));
    }

    /**
     * This route will create a parallel combo to enable move, and slide up and out together, to save time.
     * This is for position 2A and 2B only
     * @param beginP
     * @param posList
     */
    void moveAndSlideToStone(RobotPosition beginP, ArrayList<RobotPosition> posList) {
        // set up lift and slide
        SequentialComboTask slideUpExtendTask = new SequentialComboTask();
        ArrayList<RobotControl> slideUpExtendTaskList = new ArrayList<RobotControl>();
        // life up
//        slideUpExtendTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 500));

        //12/12 try to reduce the height of lift at pick up
        slideUpExtendTaskList.add(new RobotSleep(100));
        slideUpExtendTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra/2, 100));

        // slide out
        slideUpExtendTaskList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutPosAutonomous, 100));
        // open clamp side ways
        slideUpExtendTaskList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN_SIDEWAYS));
        // lift down to base
        slideUpExtendTaskList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase + 45, 100));
        slideUpExtendTask.setTaskList(slideUpExtendTaskList);

        // set up moves
        SequentialComboTask moveTask = new SequentialComboTask();
        ArrayList<RobotControl> moveTaskList = new ArrayList<RobotControl>();
        lastPos = beginP;
        for(RobotPosition p : posList) {
            PIDMecanumMoveTask oneMove = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
            oneMove.setPath(lastPos, p);
            lastPos = p;
            moveTaskList.add(oneMove);
        }
        moveTask.setTaskList(moveTaskList);
        // set up the parallel combo task list
        ArrayList<RobotControl> parallelList = new ArrayList<RobotControl>();
        ParallelComboTask moveAndDeliver = new ParallelComboTask();
        //SequentialComboTask moveAndDeliver = new SequentialComboTask(); // debug for now
        parallelList.add(moveTask);
        parallelList.add(slideUpExtendTask);
        parallelList.add(releaseWheelTask);
        moveAndDeliver.setTaskList(parallelList);
        moveAndDeliver.setTaskName("Move & Slide Ready Pickup");
        taskList.add(moveAndDeliver);
    }

    public void parking(){
        addMovement(lastPos, new RobotPosition(parkX, lastPos.getY(), finalHeading));
        lastMovement.setPower(0.8);
        lastMovement.setMinPower(0.3);
        addMovement(lastPos, new RobotPosition(parkX, parkY, finalHeading));
        lastMovement.setPower(0.8);
        lastMovement.setMinPower(0.8);
    }

    void setupDropAndMoveBuilderPlateTaskNew() {
        ArrayList<RobotControl> moveBuilderPlateTaskList = new ArrayList<RobotControl>();

        ParallelComboTask dropAndRotateTask = new ParallelComboTask();
        dropAndRotateTask.addTask(dropOffTask);  //Parallel task 1 - drop off and slide back in
        SequentialComboTask rotateBackTask = new SequentialComboTask();
        rotateBackTask.addTask(new RobotSleep(200));    // let the drop happen

        MecanumRotateTask rotate = new MecanumRotateTask(robotHardware, robotProfile, navigator);
        rotate.setRotateHeading(new RobotPosition(dropX, dropY,0), lastPos = new RobotPosition(dropX - 5, pullPlateY, -Math.PI / 2));
        rotate.setPower(0.8);
        rotate.setMinPower(0.5);
        rotateBackTask.addTask(rotate);

        // move back to the plate, push back another 10 cm just to be safe
        PIDMecanumMoveTask pushBackMove = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        pushBackMove.setPath(lastPos, new RobotPosition(dropX+10, pullPlateY, -Math.PI / 2));
        pushBackMove.setPower(0.8);
        pushBackMove.setMinPower(0.3);
        rotateBackTask.addTask(pushBackMove);
        dropAndRotateTask.addTask(rotateBackTask); // parallel task 2 - wait, rotate, and push back
        moveBuilderPlateTaskList.add(dropAndRotateTask);

        moveBuilderPlateTaskList.add(new HookPositionTask(robotHardware, robotProfile, RobotHardware.HookPosition.HOOK_ON));
        moveBuilderPlateTaskList.add(new RobotSleep(300));
        PIDMecanumMoveTask lastMove = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        lastMove.setPath(new RobotPosition(dropX+10, pullPlateY, -Math.PI / 2), new RobotPosition(dropX-35, pullPlateY, -Math.PI / 2));
        lastMove.setMinPower(0.5);
        lastMove.setPower(0.8);
        moveBuilderPlateTaskList.add(lastMove);
        
        MecanumRotateTask mecanumRotate = new MecanumRotateTask(robotHardware, robotProfile,navigator);
        mecanumRotate.setPower(0.8);
        mecanumRotate.setMinPower(0.8);
        mecanumRotate.setRotateHeading(new RobotPosition(dropX-30, pullPlateY, -Math.PI / 2), lastPos = new RobotPosition(dropX-55, afterRotatePlateY, finalHeading));
        moveBuilderPlateTaskList.add(mecanumRotate);

        moveBuilderPlateTaskList.add(new HookPositionTask(robotHardware, robotProfile, RobotHardware.HookPosition.HOOK_OFF));
        moveBuilderPlateTaskList.add(new RobotSleep(100));

        moveBuilderPlateTask = new SequentialComboTask();
        moveBuilderPlateTask.setTaskList(moveBuilderPlateTaskList);
        moveBuilderPlateTask.setTaskName("Move & Rotate BuilderPlate");
        // Notice the lastPos is set after the final rotate for parking
    }

    public ArrayList buildPickUpFirstBlockAndPark(){
        if(this.delay > 0){   // add 1 sleep anyway for debug
            taskList.add(new RobotSleep(delay * 1000 ));
        }

        ArrayList<RobotPosition> moveToStonePos = new ArrayList<RobotPosition>();

        populateCommonTask();
//        stoneX = 90;
//        switch (startingPositionModes){
//            case ("RED_2") :
//                parkY = -85;
//                parkWall = 10;
//                parkNeutral = 50;  //60 robot after long distance travel, angle tilted, almost hit the platform, reduce x to avoid crash
//                stoneY = 46;  // or 44
//                bridgeY = -110;
//                break;
//            case("BLUE_2"):
//                parkY = 98;
//                parkWall = 10; //50;
//                parkNeutral = 60; //120;
//                stoneY = -38;
//                bridgeY = 125;
//                Logger.logFile("stoneY = " + stoneY);
//                break;
//       }
        moveToStonePos.add(new RobotPosition(0,0,0));
        moveToStonePos.add(new RobotPosition(0, stoneY, 0));
        moveToStonePos.add(new RobotPosition(stoneX, stoneY, 0));
        moveToStonePos.add(new RobotPosition(stoneX-20, stoneY, 0));
        moveToStonePos.add(new RobotPosition(stoneX-20, stoneY, 0));
        moveToStonePos.add(new RobotPosition(stoneX-20, stoneY-38, 0));

        moveAndSlideToStone(new RobotPosition(0, 0, 0), moveToStonePos);
        taskList.add(new ClampAngleRotationTask(robotHardware, robotProfile, RobotHardware.ClampAnglePosition.BACK));
        taskList.add(pickUpTask);
        taskList.add(new ClampAngleRotationTask(robotHardware, robotProfile, RobotHardware.ClampAnglePosition.NORMAL));

        //*****store stone sequence****
        ArrayList<RobotControl> storeStoneList = new ArrayList<RobotControl>();
        //slide in
        storeStoneList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 500));
        // lift down
        storeStoneList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftHomeGrabPos,500));
        SequentialComboTask storeStoneTask = new SequentialComboTask();
        storeStoneTask.setTaskList(storeStoneList);

        //****prepare for drop off after bridge line****
        ArrayList <RobotControl> liftAndExtendList = new ArrayList<RobotControl>();
        liftAndExtendList.add(new WaitForNavigationTask(navigator, new RobotPosition(-10, bridgeY, -1), new RobotPosition(100, dropY, 1)));

        //12/12, reduce the height of lift at drop off
        liftAndExtendList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftPerStone*3/2, 500));

        liftAndExtendList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOutMax, 500));
        SequentialComboTask liftAndExtendTask = new SequentialComboTask();
        liftAndExtendTask.setTaskList(liftAndExtendList);

        //****move across the bridge line***
         ArrayList<RobotControl> crossBridgeLineList = new ArrayList<RobotControl>();
        PIDMecanumMoveTask moveToRoute = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        moveToRoute.setPath(new RobotPosition(stoneX, stoneY, 0), lastPos = new RobotPosition(deliverRoute(deliverRoute), stoneY, 0));
        crossBridgeLineList.add(new RobotSleep(1000));  //12/12, too fast and lift is almost hitting the bridge bar all the time
        crossBridgeLineList.add(moveToRoute);
        PIDMecanumMoveTask moveAcross = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        moveAcross.setPath(lastPos, lastPos = new RobotPosition(deliverRoute(deliverRoute), dropY, 0));
        moveAcross.setPower(0.8);
        crossBridgeLineList.add(moveAcross);
        SequentialComboTask crossBridgeLineTask = new SequentialComboTask();
        crossBridgeLineTask.setTaskList(crossBridgeLineList);

        //****combine all****
        ParallelComboTask moveAndExtendTask = new ParallelComboTask();
        ArrayList<RobotControl> moveAndExtendList = new ArrayList<RobotControl>();
        moveAndExtendList.add(storeStoneTask);
        moveAndExtendList.add(liftAndExtendTask);
        moveAndExtendList.add(crossBridgeLineTask);
        moveAndExtendTask.setTaskList(moveAndExtendList);
        taskList.add(moveAndExtendTask);
        taskList.add(dropOffTask);
        parking();
        return taskList;
    }

    void populateCommonTask(){
        stoneX = 68;
        dropX = 73;
        parkWall = 10; //50;
        parkNeutral = 60; //120;
        pullPlateX = -10;
        finalPushX = 50;
        int ySign;

        if (startingPositionModes.contains("RED")) {
            ySign = -1;
            finalHeading = 0;
        }
        else {
            ySign = 1;
            finalHeading = -Math.PI;
        }

        switch (startingPositionModes){
            case ("RED_2") :
                dropY = -210; //-180
                parkY = -120;
                stoneY = 28 + (2 - skyStonePosition) *20;
                bridgeY = -110;
                if (skyStonePosition==0) {
                    stoneY -= 20;   // special situation, we may want to use intake to pick it up
                }
                break;
            case("RED_3"):
                dropY = -160; //-130
                parkY = -50;
                stoneY = 26 + (2 - skyStonePosition) *20;
                bridgeY = -50;
                break;
            case("RED_5"):
                dropY = -160 + 120; //-130
                parkY = -50 + 120;
                //bridgeY = -50 + 120;
                break;
            case("BLUE_2"):
                dropY = 230; //210. used to be 220 on 2020.01.10
                parkY = 115;
                stoneY = -80 + skyStonePosition * 20;   //83
                if (skyStonePosition==0) {
                    stoneY += 20;   // special situation, we may want to use intake to pick it up
                }
                bridgeY = 125;
                break;
            case("BLUE_3"):
                dropY = 175;  //150 //12/12 set drop off block at further distance and back up create space to hook. used to be 170 on 2020.01.10
                parkY = 55 ;
                stoneY = -38 + (skyStonePosition-2) * 20;
                bridgeY = 65;
                break;
            case("BLUE_5"):
                dropY = 175 - 120;  //150 //12/12 set drop off block at further distance and back up create space to hook. used to be 170 on 2020.01.10
                parkY =55 - 120 ;
                //bridgeY = 65 - 120;
                break;
        }

        if(parking.equals("BridgeWall")) {
            parkX = parkWall;
        }
        else {
            parkX = parkNeutral;
        }
        pullPlateY = dropY;
        finalPushY = dropY - ySign * 50;
        afterRotatePlateY = pullPlateY - ySign * 40;
        // now we can initialized these after the parameters are set above
        setupPickUpTask();
        setupDropOffTask();
        setupReleaseWheelTask();
    }
}