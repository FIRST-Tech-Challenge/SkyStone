package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "SkystoneOnly", group = "Concept")
//@Disabled
public class SkystoneNew extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //motors
    private DcMotor mtrFR = null;
    private DcMotor mtrFL = null;
    private DcMotor mtrBL = null;
    private DcMotor mtrBR = null;

    //servos
    private Servo grab = null;
    private Servo arm = null;
    private Servo extend = null;
    private Servo FL_hook = null;
    private Servo FR_hook = null;
    private Servo LED_strip = null;

    //switches
    private DigitalChannel alliance_switch;
    private DigitalChannel position_switch;

    //colorsensor
    private NormalizedColorSensor color_sensor;

    //webcam
    private WebcamName Webcam1 = null;

    //constants
    double colorRed = 0.6695;
    double colorBlue = 0.7445;
    double colorBlack = 0.7745;
    double colorWhite = 0.7595;

    double runtime1 = 0;

    int strafe_Swapper = 1;
    double alliance = 0;
    final double red = 1;
    final double blue = -1;
    int ParkingSpot = 0;
    int insideLane = 0;
    int outsideLane = 1;
    double DistanceStrafed = 0;

    long roundedticks = 0;
    private final double ticksPerMm = 1.68240559922;
    int positionMm = 0;

    int ticksPerInchRound = 42;
    double halfPower = 0.5;
    int turnPull = 55*ticksPerInchRound;

    double groundArm = 0.05;
    double retractArm = 0.75;
    double fullGrab = 0.33;
    double releaseGrab = 0.7;
    double FLup = 0.8;
    double FRup = 0.57;
    double FLdown = 1;
    double FRdown = 0.27;


    //measured center of bridge to center of closest wheel
    double robotToBridge = 410;
    //webcam aligned with center of first two stones
    //blue side 660mm offset
    //red side 410mm offset

    //measured center of bridge to center of wheel at desired deposit location
    double bridgeToDeposit = 1150;
    //blue side 1300mm to foundation deposit
    //red side 1150mm to foundation deposit


    double webcamToArmMm = 310;

    private int ticksStrafed = 0;

    //vuforia settings

    //vuforia constants
    boolean targetVisible = false;
    boolean skystone_Align = false;
    boolean detection_Complete = false;
    double ycoordinate = 0;
    float phoneXRotate = 0;
    float phoneYRotate = 0;
    float phoneZRotate = 0;

    //conversion for skystone
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;
    private static final float stoneZ = 2.00f * mmPerInch;

    //class members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    @Override
    public void runOpMode() {
        //configuration of robot stuff
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        //motors
        mtrFR = hardwareMap.get(DcMotor.class, "rightFront_drive");
        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrFL = hardwareMap.get(DcMotor.class, "leftFront_drive");
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrBL = hardwareMap.get(DcMotor.class, "leftBack_drive");
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        mtrBR = hardwareMap.get(DcMotor.class, "rightBack_drive");
        mtrBR.setDirection(DcMotor.Direction.REVERSE);

        //servos
        grab = hardwareMap.get(Servo.class, "grab");
        grab.setDirection(Servo.Direction.REVERSE);

        arm = hardwareMap.get(Servo.class, "arm");

        extend = hardwareMap.get(Servo.class, "extend");

        FL_hook = hardwareMap.get(Servo.class, "FL_hook");
        FL_hook.setDirection(Servo.Direction.REVERSE);

        FR_hook = hardwareMap.get(Servo.class, "FR_hook");
        FR_hook.setDirection(Servo.Direction.REVERSE);

        //alliance and position switches
        alliance_switch = hardwareMap.get(DigitalChannel.class, "alliance_switch");
        position_switch = hardwareMap.get(DigitalChannel.class, "position_switch");

        //LEDs
        LED_strip = hardwareMap.get(Servo.class, "LED_strip");

        //color sensor
        color_sensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        //set hook positions
        FL_hook.setPosition(FLup);
        FR_hook.setPosition(FRup);

        //webcam
        Webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //vuforia configuration
        parameters.vuforiaLicenseKey = "AQsl+Kb/////AAABmbE31v+dqUMNrHZvXmTH1TYMKPOzGdwPeKVPFFdmQC1IslYsLtjPjtKBkJ0UYlfipscEJ+KhpMxZshIN22jTxZIKJp/CIxjik5UGibWOLsfPTkMIX2WFXb7uJBFeEUr3kqLZWmrf5sAkMa9B5HNOXGKeYyqOFRBht5k0MrFIrYZAnmha98dsxodvP8TPkHY/EHy57K2ww9TCqEstOJVf6DtJO+zgMEJz8iv2ASr3Mc6RFwNvS/l1Gsq3RmpEzK3/BasvQ0gakJO4zUlMZ5+CDHchljWOcUTdls6dSeyfMxv7kkLStV018qcq15bTEK17lEbmPU7fcf97/Sp7YBe89Kw/CrVRmHQWMgSmrc+6NG6T";
        parameters.cameraName = Webcam1;
        parameters.cameraDirection = CAMERA_CHOICE;

        if (CAMERA_CHOICE == BACK)
        {phoneYRotate = -90;}
        else
        {phoneYRotate = 90;}

        if (PHONE_IS_PORTRAIT)
        {phoneXRotate = 90;}

        final float CAMERA_FORWARD_DISPLACEMENT = 8.5f * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 3.5f * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT = -4.75f * mmPerInch;

        //vuforia init
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        //trackable object:
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        //set stone target position
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        //let trackables know where the phone is
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        //vuforia configuration stops here


        //determine side
        if (alliance_switch.getState() == true) {
            telemetry.addData("Alliance:", "Red");
            strafe_Swapper = 1;
            alliance = red;
            LED_strip.setPosition(colorRed);
        } else {
            telemetry.addData("Alliance", "Blue");
            strafe_Swapper = -1;
            alliance = -1;
            LED_strip.setPosition(colorBlue);
        }
        if (position_switch.getState() == true) {
            telemetry.addData("Innermost spot:", "### ___");
            ParkingSpot = insideLane;
        } else {
            telemetry.addData("Outer Spot", "___ ###");
            ParkingSpot = outsideLane;
        }

        telemetry.addData("LED Status:", "On");
        telemetry.update();

        //set the LEDs to white to let vuforia search
        LED_strip.setPosition(colorWhite);
        targetsSkyStone.activate();

        arm.setPosition(retractArm);
        grab.setPosition(fullGrab);


        /**

         Code begins here

         */


        waitForStart();

        runtime.reset();
        //open claw
        arm.setPosition(groundArm);
        grab.setPosition(releaseGrab);

        //drive straight forward
        distanceForward(370, 0.25);
        speedStrafe(-0.16);

        runtime.reset();
        while (skystone_Align == false) {
            while (!isStopRequested()) {
                detection_Complete = false;
                skystone_Align = false;
                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;
                runtime1 = runtime.time();
                telemetry.addData("Time run:", runtime1);


                //if it sees the stone, set target visible to true
                for (VuforiaTrackable trackable : targetsSkyStone) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

                //if the webcam see the stone:
                if (targetVisible) {

                    for (VuforiaTrackable trackable : targetsSkyStone) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }

                    brakeMotors();
                    sleep(500);


                    //robot position
                    VectorF translation = lastLocation.getTranslation();

                    //ycoordinate rounded
                    ycoordinate = translation.get(1);
                    telemetry.addData( "{Y} = %.1f", ycoordinate);
                    telemetry.update();

                    ticksStrafed = ticksStrafed + mtrFR.getCurrentPosition();

                    //strafe
                    runtime.reset();


                    distanceStrafe((ycoordinate) - webcamToArmMm,0.3);
                    //distanceStrafeWithWait(-(ycoordinate + webcamToArmMm), -0.3, 3);

                    //it saw the stone, so:
                    detection_Complete = true;


                    if (!mtrFR.isBusy()) {
                        telemetry.addData("y-coordinate:", ycoordinate);
                        telemetry.update();
                        LED_strip.setPosition(colorBlack);
                        telemetry.addData("LED Status:", "Off");
                        telemetry.update();
                        skystone_Align = true;
                        break;
                    }
                }
                //if it doesn't see the stone:
                else {
                    telemetry.addData("Visible Target", "none");
                    telemetry.addData("Skystone", "None");
                }

                telemetry.addData("Time Run:", runtime);
                telemetry.update();
            }

            if (skystone_Align = true) {
                break;
            }
            telemetry.update();
        }

        //first skystone found
        ticksStrafed = mtrBR.getCurrentPosition()+ticksStrafed;
        //ticksStrafed = Math.abs(mtrBR.getCurrentPosition())+ticksStrafed;
        DistanceStrafed = -ticksStrafed / ticksPerMm;

        telemetry.addData("Distance strafed: ", DistanceStrafed);
        detection_Complete = true;
        telemetry.addData("Skystone:", "Detected");
        telemetry.update();

        //drive forward after aligned
        distanceForward(250,0.3);
        brakeMotors();

        //grab
        grab.setPosition(fullGrab);
        waitFor(0.75);

        //retract
        arm.setPosition(retractArm);
        telemetry.addData("Arm:", "Retracted");
        telemetry.update();

        //backwards
        distanceForward(-30, -0.5);

        if(alliance == red){
            robotToBridge = 410;
            bridgeToDeposit = 1600;
        }
        else{
            robotToBridge = 660;
            bridgeToDeposit = 1200;
        }

        if(alliance == red){
            distanceStrafe((-DistanceStrafed + robotToBridge + bridgeToDeposit) * strafe_Swapper, 0.5);
        }
        else {
            distanceStrafe((DistanceStrafed + robotToBridge + bridgeToDeposit) * strafe_Swapper, 0.5);
        }

        distanceForward(80, 0.3);

        arm.setPosition(groundArm);
        waitFor(0.75);
        grab.setPosition(releaseGrab);
        waitFor(0.75);
        arm.setPosition(retractArm);
        waitFor(0.25);
        grab.setPosition(fullGrab);

        if (ParkingSpot == insideLane) {
            distanceForward(-40, -0.5);
            distanceStrafe((-950 * strafe_Swapper), 0.4);
        } else {
            distanceForward(-600, 0.4);
            distanceStrafe((-950 * strafe_Swapper), 0.4);
        }
        brakeMotors();

        /**

         Code ends here

         */

    }


    private void resetEncoders() {
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void runToPosition() {
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void runWithoutEncoder() {
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void runUsingEncoders() {
        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void brakeMotors() {
        mtrFL.setPower(0);
        mtrFR.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);
    }
    private void isBusy() {
        while (mtrBR.isBusy()){
        }
    }
    private void waitFor(double waittime) {
        runtime.reset();
        while (runtime.time() < waittime) {
        }
    }

    private void forward(double power) {
        mtrFR.setPower(power);
        mtrFL.setPower(power);
        mtrBL.setPower(power);
        mtrBR.setPower(power);
    }
    private void forwardPosition(double position) {
        positionMm = (int) (position * ticksPerMm);
        mtrFR.setTargetPosition(positionMm);
        mtrFL.setTargetPosition(positionMm);
        mtrBR.setTargetPosition(positionMm);
        mtrBL.setTargetPosition(positionMm);
    }
    private void distanceForward(double position, double power) {
        if(position<0){
            power = -power;
        }
        resetEncoders();
        forwardPosition(position);
        runToPosition();
        forward(power);
        isBusy();
        //wait for motion to finish
        brakeMotors();
        runWithoutEncoder();
    }

    private void strafe(double power) {
        mtrFR.setPower(-power * strafe_Swapper);
        mtrFL.setPower(power * strafe_Swapper);
        mtrBL.setPower(-power * strafe_Swapper);
        mtrBR.setPower(power * strafe_Swapper);
    }
    private void strafePosition(double position){
        positionMm = (int) (position * ticksPerMm);
        mtrFR.setTargetPosition(-positionMm);
        mtrFL.setTargetPosition(positionMm);
        mtrBL.setTargetPosition(-positionMm);
        mtrBR.setTargetPosition(positionMm);
    }
    private void distanceStrafe(double position, double power) {
        resetEncoders();
        strafePosition(position);
        runToPosition();
        strafe(power);
        isBusy();
        brakeMotors();
        runWithoutEncoder();
    }
    private void distanceStrafeWithWait(double position, double power, double time) {
        Range.clip(power, -1, 1);
        resetEncoders();
        strafePosition(position);
        runToPosition();
        strafe(power);
        while (mtrFR.isBusy()) {
            if(runtime.time()>time){
                break;
            }
        }
        brakeMotors();
        runWithoutEncoder();
    }
    private void strafePower(double power) {
        mtrFR.setPower(-power* strafe_Swapper);
        mtrFL.setPower(power * strafe_Swapper);
        mtrBL.setPower(-power * strafe_Swapper);
        mtrBR.setPower(power * strafe_Swapper);
    }
    private void speedStrafe(double power) {
        resetEncoders();
        runUsingEncoders();
        strafePower(power);
        runWithoutEncoder();
    }


}




