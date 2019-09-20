package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our autonomous program for the depot side on the blue side of the field. This program runs
 * without the phone light for Tensor Flow. This is the go to program. This program... .
 */
@Autonomous(name = "Skystone Blue Depot No Light", group = "Beep")
public class TestingTensorFlowObjectDetectionNoLight extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    //Calling our hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calling the Library Gyro program to use the gyro turn function
    LibraryGyro gyroTurn = new LibraryGyro();
    // Calling the Library Gyro Drive program to use the gyro drive function
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    // Calling the Library Grid Nav Library to use the grid navigation functions
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();

    // Calling the Library Tensor Flow No Light to use the Tensor Flow function without
    LibraryTensorFlowObjectDetectionNoLight tensorFlow =
            new LibraryTensorFlowObjectDetectionNoLight(robot, telemetry);
    // Declaring skystone position value to read what position Tensor Flow sees the skystone position
    String skystonePosition = "";

    /**
     * This method is the main body of our code which contains the set of commands carried out in our crater side autonomous program.
     */
    @Override
    public void runOpMode() {


        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        //initializing the hardware map
        robot.init(hardwareMap);
        //initializing the grid Nav function
        gridNavigation.init(robot, gyroTurn, telemetry);
        //initializing the gyro turn function
        gyroTurn.init(robot, telemetry);
        //initializing the gyro drive function
        gyroDrive.init(robot, telemetry, robot.rightBack);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        /*
         * Use ultrasonic to read south wall
         * if > 1 set y pos to value
         * if < 1 set grid pos to (0.5,2.5)
         */
        // Set initial Grid Nav position
        gridNavigation.setGridPosition(.6076, .6076, 45);

        // Start up Tensor Flow to read skystone position while landing
        getSkystonePos();

        //wait for start
        waitForStart();

        int X = 0;
        int Y = 1;
        /*
         * UPDATE GRID NAV WITH END ANGLE
         */
        //int END_ANGLE = 2;

        // Skystone pos 1
        double[] SKYSTONE_POS_1 = {1.5, 1.75}; /* END_ANGLE = 0 */
        // Skystone pos 2
        double[] SKYSTONE_POS_2 = {1.5, 1.5}; /* END_ANGLE = 0 */
        // Skystone pos 3
        double[] SKYSTONE_POS_3 = {1.5, 1.25}; /* END_ANGLE = 0 */
        // Foundation pos
        double[] FOUNDATION_POS = {1.75, 5}; /* END_ANGLE = 0 */
        // Repositioning pos
        double[] REPOSITIONING_POS = {0.5, 5}; /* END_ANGLE = 0 */
        // Parking pos
        double[] PARKING_POS = {0.5, 3}; /* END_ANGLE = -90 */

        // This is a switch block that plays the program in relation to the skystone position
        // Tensor Flow reads
        switch (skystonePosition) {

            // If Tensor Flow reads the first skystone position then it plays this case
            case "Pos 1":
                telemetry.addData("Telemetry", "Skystone Pos = Pos 1");
                printTelemetry(20);
                if (skystonePosition == "Pos 1") {
                    gridNavigation.driveToPosition(SKYSTONE_POS_1[X], SKYSTONE_POS_1[Y], .5);
                    telemetry.addData("Grid Nav Go to Pos X", SKYSTONE_POS_1[X]);
                    telemetry.addData("Grid Nav Go to Pos Y", SKYSTONE_POS_1[Y]);

                    /*
                     * PICK UP SKYSTONE
                     */
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }
                break;

            // If Tensor Flow reads the second skystone position then it plays this case
            case "Pos 2":
                telemetry.addData("Telemetry", "Skystone Pos = 2");
                printTelemetry(40);
                if (skystonePosition == "Pos 2") {
                    gridNavigation.driveToPosition(SKYSTONE_POS_2[X], SKYSTONE_POS_2[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", SKYSTONE_POS_2[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", SKYSTONE_POS_2[Y]);
                    /*
                     * PICK UP SKYSTONE
                     */
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }
                break;

            // If Tensor Flow reads the third skystone position then it plays this case
            case "Pos 3":
                telemetry.addData("Telemetry", "Skystone Pos = 3");
                printTelemetry(60);
                if (skystonePosition == "Pos 3") {
                    telemetry.addData("Grid Nav Goto Pos X", SKYSTONE_POS_3[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", SKYSTONE_POS_3[Y]);
                    // drive to the third skystone position
                    gridNavigation.driveToPosition(SKYSTONE_POS_3[X], SKYSTONE_POS_3[Y], .5);
                    /*
                     * PICK UP SKYSTONE
                     */
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(70);
                }
                break;
            // should never get to this case but in case it can't find the skystone position
            // it goes to this default case
            default:
                telemetry.addData("Telemetry", "Didn't see skystone pos");
                telemetry.update();
                gridNavigation.driveToPosition(SKYSTONE_POS_1[X], SKYSTONE_POS_1[Y], .5);
                break;
        }
        // drive to foundation to deposit skystone
        gridNavigation.driveToPosition(FOUNDATION_POS[X], FOUNDATION_POS[Y], .5);
        /*
         * PLACE SKYSTONE
         */
        /*
         * LOWER FOUNDATION ATTACHMENT
         */
        // drive to reposition foundation
        gridNavigation.driveToPositionBackwards(REPOSITIONING_POS[X], REPOSITIONING_POS[Y], .7);
        // parking under alliance sky bridge
        gridNavigation.driveToPosition(PARKING_POS[X], PARKING_POS[Y], .5);


    }

    /**
     * This method prints telemetry for our autonomous program
     *
     * @param codePos This is the value we use in telemetry to see where in the code we are
     */
    private void printTelemetry(int codePos) {
        telemetry.addData("skystone Pos", skystonePosition);
        telemetry.addData("Code Position", codePos);
        telemetry.update();
    }

    /**
     * This method calls Tensor Flow in order to read the skystone position
     */
    public void getSkystonePos() {
        int debounceCount = 0;
        long startTime = 0;
        String previousPosition;
        /*
         * UPDATE WITH NEW REPOSITORY
         */
        skystonePosition = tensorFlow.findSkystone();

        // Switch block that indicated which skystone position it reads
        switch (skystonePosition) {
            case ("Pos 1"):
                telemetry.addData("Telemetry", "Pos 1");
                telemetry.update();
                break;
            case ("Pos 2"):
                telemetry.addData("Telemetry", "Pos 2");
                telemetry.update();
                break;
            case ("Pos 3"):
                telemetry.addData("Telemetry", "Pos 3");
                telemetry.update();
                break;

            // If it reads unknown than it goes to this default case
            default:
                telemetry.addData("Telemetry", "Unknown Position");
                telemetry.update();
                // sets skystone pos to center as default
                skystonePosition = "Pos 1";
                break;
        }

        telemetry.update();
    }
}