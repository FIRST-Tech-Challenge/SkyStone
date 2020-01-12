package org.firstinspires.ftc.teamcode;

/**
 * 2019.10.26
 * Athena Z.
 */

public class BuildingPlateMoveTask implements RobotControl {
    private static double MAX_PLATE_DIST = 50.0;    // anything less than 50 means that's plate's edge
    private static double DELIVER_FROM_RIGHT_EDGE = 14; //cm
    private static double DELIVER_FROM_LEFT_EDGE = 32;  //cm
    private static double DELIVER_DIST_TO_PLATE = 10;    //cm
    private static double MAX_APPROACH_DIST = 50;       //cm
    private static int MAX_SAMPLE = 100;                //max array size
    private static int MAX_SAMPLE_DIST = 15;            //cm
    ApproachFrom approachFrom;
    RobotNavigator navigator;
    RobotHardware robot;
    RobotProfile profile;
    PIDMecanumMoveTask pidMove;
    MecanumRotateTask rotMove;
    TaskState currState;
    // distance measure
    double edgeY;
    double[] distHist;
    double[] yHist;
    int ndxHist;
    /**
     * After driver place the robot right outside of the plate, this task can be triggered to move it into the
     * delivery position.
     * 1. First move toward the plate, keep checking the distance (assume things > 50cm is not plate)
     * 2. Once the plate's edge is detected, mark it down, keep going
     * 3. Once move for enough samples (by distance or sample count), stop
     * 4. Use the distance sample to average the approach angle, and use the plates edge location to calculate the
     *    delivery position
     * 5. Use MecanumRotateMoveTask to move to the target location and angle for delivery
     *
     * @param robot
     * @param profile
     * @param navigator
     * @param approachFrom
     */

    public BuildingPlateMoveTask(RobotHardware robot, RobotProfile profile,RobotNavigator  navigator, ApproachFrom approachFrom) {
        this.approachFrom = approachFrom;
        this.robot = robot;
        this.profile = profile;
        this.navigator = navigator;
        distHist = new double[MAX_SAMPLE];
        yHist = new double[MAX_SAMPLE];
    }

    public String toString() {
        return "Building Plate Move from " + approachFrom;
    }

    @Override
    public void prepare() {
        // reset navigator
        navigator.setInitPosition(0, 0,0);
        pidMove = new PIDMecanumMoveTask(robot, profile, navigator);
        // for now, just do approach from RIGHT (heading = 0 and move up)
        pidMove.setMinPower(0.2);
        pidMove.setPower(0.2);
        pidMove.setRelativePath(0, 1000);
        // final approach task
        rotMove = new MecanumRotateTask(robot, profile, navigator);
        rotMove.setMinPower(0.3);
        rotMove.setPower(0.3);
        // init other variables
        edgeY = 0;
        ndxHist = 0;
        currState = TaskState.APPROACH;
        pidMove.prepare();
    }

    @Override
    public void execute() {
        // first measure distance
        if (currState==TaskState.APPROACH) {
            double dist = robot.getRightDistance();
            if (dist < MAX_PLATE_DIST) {
                // this is the edge of the plate, transition to next state
                edgeY = navigator.getWorldY();
                yHist[ndxHist] = edgeY;
                distHist[ndxHist] = dist;
                ndxHist++;
                currState = TaskState.GET_ANGLE;
            }
            if (Math.abs(navigator.getWorldY()) > MAX_APPROACH_DIST) {
                // aboard mission
                currState = TaskState.DONE;
                pidMove.cleanUp();
            }
            else {
                pidMove.execute();
            }
        }
        else if (currState == TaskState.GET_ANGLE) {
            double dist = robot.getRightDistance();
            yHist[ndxHist] = navigator.getWorldY();
            distHist[ndxHist] = dist;
            ndxHist++;
            if (Math.abs(navigator.getWorldY()-edgeY) > MAX_SAMPLE_DIST || ndxHist==MAX_SAMPLE) {
                pidMove.cleanUp();  // stop the motor
                prepareFinalApproach();
                currState = TaskState.FINAL_APPROACH;
            }
            else {
                pidMove.execute();
            }
        }
        else if (currState == TaskState.FINAL_APPROACH) {
            rotMove.execute();
            if (rotMove.isDone()) {
                rotMove.cleanUp();
                currState = TaskState.DONE;
            }
        }
    }

    /**
     * Use the yHist and distHist to calculate the angle the final destination and angle
     */
    void prepareFinalApproach() {
        //looks like the sensor is not accurate at the edge, so we only use that to detect the edge
        // but use the last 10 sample to calculate the angle average
        double angleTot = 0;
        int cnt = 0;
        for(int i=ndxHist-11; i<ndxHist-1; i++) {
            angleTot = angleTot + Math.atan2(distHist[i] - distHist[ndxHist-1], yHist[ndxHist-1] - yHist[i]);
            cnt++;
        }
        double angle = angleTot/cnt;   // this is the new heading for robot that
        double edgeD = Math.tan(angle) * (yHist[ndxHist-1] - yHist[0]) + yHist[ndxHist-1]; //estimate the dist when hit the edge
        double finalY = DELIVER_FROM_LEFT_EDGE * Math.cos(angle) - DELIVER_DIST_TO_PLATE * Math.sin(angle) + yHist[0];
        double finalX = edgeD - DELIVER_FROM_LEFT_EDGE * Math.sin(angle) - DELIVER_DIST_TO_PLATE * Math.cos(angle);
        rotMove.setRotateHeading(new RobotPosition(navigator.getWorldX(), navigator.getWorldY(), navigator.getHeading()),
                new RobotPosition(finalX,finalY, -angle));
        Logger.logFile("PlateApproach - y0: " + edgeY + " d0: " + distHist[0] + " yLast: " + yHist[ndxHist-1] + " dLast: " + distHist[ndxHist-1] + " ndx: " + ndxHist);
        Logger.logFile("PlateApproach - final: " + finalX + ", " + finalY + " angle: " + angle);
        for(int i=0; i<ndxHist; i++) {
            Logger.logFile("," + yHist[i] + "," + distHist[i]);
        }
        rotMove.prepare();
    }

    @Override
    public void cleanUp() {

    }

    @Override
    public boolean isDone() {
        return currState==TaskState.DONE;
    }

    public enum ApproachFrom { LEFT, RIGHT }

    private enum TaskState { APPROACH, GET_ANGLE, FINAL_APPROACH, DONE }
}
