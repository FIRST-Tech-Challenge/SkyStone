package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.parts.AwesomeArm;
import org.firstinspires.ftc.teamcode.parts.ManyMotors;
import org.firstinspires.ftc.teamcode.parts.SeveralServos;
import org.firstinspires.ftc.teamcode.parts.WeirdWheels;
import org.firstinspires.ftc.teamcode.util.glob.SharedTelemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Autonomy extends LinearOpMode {
    boolean parkBridgeSide     = false;
    WeirdWheels.Side side      = WeirdWheels.Side.LEFT;
    boolean execCollectStones  = false;
    boolean execMoveFoundation = false;

    private WeirdWheels wheels;

    private ManyMotors intake;
    private boolean intakeBtn;
    private boolean intakeOpen;

    private AwesomeArm arm;
    private boolean grippersBtn;

    private SeveralServos bulldozer;

    private int dToFoundation           = 0; // TODO: add y value to move wheels TO foundation
    private int dToBelowFoundation      = 0; // TODO: add x value to go below foundation
    private int dToBottomFromResetPos   = 0; // TODO: add distance to bottom from reset position
    private int dToBottomFromLoadingPos = 0; // TODO: add distance to bottom from load only position
    private int dToStone                = 0; // TODO: add distance to approach skystone

    private List<VuforiaTrackable> allTrackables;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //registerMotors();
        registerVuforia();

        telemetry.addData("Robob says","I'm Ready!");
        telemetry.update();

        // After we are done initializing our code, we wait for Start button.
        waitForStart();

        if(execMoveFoundation) moveFoundation();
        if(execCollectStones) collectStones();
    }

    public void registerVuforia() {

        final String VUFORIA_KEY =
                "AR23ZAD/////AAABmRQ39GR6pk5noX6VjfkWqkQrfstU6L6wCyclUiJChoh4AjSZ+8sBCWr7kooyawF+1YArwQSfZ5FPazPgEPN3aJCdA5ouzDJtlPjl9MKLT9PMdEsyJCtxxhgVJPVcK59dYh/23JPmDyOihWfHzuzJtXjWF8jZfBverhcvfIXaleAr2u7Zq47hAs/m9jRQb2f4PwNnYdvx6lcqm1JNM1pLNnl2ckA4H5zf4Skdme5EdFLxQ+31k/nQSZxq+P3FmvxB7qML8GSJczWdOu+n5SL4ZqeJYCmS1Ur8XFK5H+KmK1EobTi2p0Hn6K3vyJ4MZzCWnsOvKQRFqWva/ty4CmXoqdBpojYWFlDwoSMshIntv5yS";

        final float mmPerInch = 25.4f;

        final float stoneZ = 2.00f * mmPerInch;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = BACK;

        // Class Members
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        float phoneXRotate    = 0;
        float phoneYRotate    = -90;
        float phoneZRotate    = 0;

        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        allTrackables = new ArrayList<VuforiaTrackable>(targetsSkyStone);

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        //  Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsSkyStone.activate();
    }

    public void registerMotors() {
        SharedTelemetry.telemetry = telemetry;

        telemetry.addData("Status", "Initialized");

        // wheels
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        List<DcMotor> wheelList = new ArrayList<>(Arrays.asList(fl, fr, bl, br));
        this.wheels = new WeirdWheels(wheelList);
        this.wheels.setSide(this.side);

        // intake
        DcMotor i1 = hardwareMap.get(DcMotor.class, "i1");
        DcMotor i2 = hardwareMap.get(DcMotor.class, "i2");
        List<DcMotor> intakeList = new ArrayList<>(Arrays.asList(i1,i2));
        this.intake = new ManyMotors(intakeList);

        // arm
        DcMotor shoulder     = hardwareMap.get(DcMotor.class, "shoulder");
        DcMotor actuator     = hardwareMap.get(DcMotor.class, "actuator");
        Servo elbow          = hardwareMap.get(Servo.class, "elbow");
        Servo lGrip          = hardwareMap.get(Servo.class, "lGrip");
        Servo rGrip          = hardwareMap.get(Servo.class, "rGrip");
        List<Servo> grippers = new ArrayList<>(Arrays.asList(lGrip,rGrip));
        this.arm = new AwesomeArm(shoulder,actuator,elbow,grippers);

        Servo lbd = hardwareMap.get(Servo.class, "lbd");
        Servo rbd = hardwareMap.get(Servo.class, "rbd");
        List<Servo> bdList = new ArrayList<>(Arrays.asList(lbd,rbd));
        this.bulldozer = new SeveralServos(bdList);
    }

    public void moveFoundation() {

        // go to foundation
        wheels.moveTo(0,dToFoundation);
        while(wheelsBusy()) {
            telemetry.addData("Robob is","moving to foundation");
        }

        // bring down bulldozer
        bulldozer.setPosition(new ArrayList<>(Collections.nCopies(2, 1.0)));

        // move foundation back
        wheels.moveTo(0,-dToFoundation);
        while(wheelsBusy()) {
            telemetry.addData("Robob is","moving foundation to corner");
        }

        // go below foundation
        wheels.moveTo(dToBelowFoundation,0);
        while(wheelsBusy()) {
            telemetry.addData("Robob is","moving below foundation");
        }

        // go to reset position
        wheels.moveTo(0, dToFoundation);
        while(wheelsBusy()) {
            telemetry.addData("Robob is","moving to reset position");
        }
    }

    public void collectStones() {
        if(execMoveFoundation) {
            wheels.moveTo(dToBottomFromResetPos,0); // TODO: verify distances
        }
        else {
            wheels.moveTo(dToBottomFromLoadingPos, dToFoundation); // TODO: verify distances
        }

        while(wheelsBusy()) {
            telemetry.addData("Robob is","preparing to collect stones");
        }

        int stonesFound = 0;
        while (stonesFound < 2) {
            int origTicks = wheels.motors.get(0).getCurrentPosition();
            wheels.setVelocityX(-0.2);
            wheels.moveMecanum();
            for (VuforiaTrackable trackable : allTrackables) {
                if (isStone(trackable)) {
                    int newTicks = wheels.motors.get(0).getCurrentPosition();
                    int difTicks = Math.abs(origTicks - newTicks);
                    stonesFound++;
                    wheels.stop();
                    VuforiaTrackableDefaultListener stone = (VuforiaTrackableDefaultListener)trackable.getListener();
                    int z = zAngle(stone);
                    while(z != 0) {
                        wheels.setVelocityX(-0.1);
                        wheels.moveMecanum();
                        z = zAngle(stone);
                    }
                    swallowStone();
                    wheels.moveTo(-(dToBottomFromResetPos-difTicks),0);
                    pukeStone();
                    wheels.moveTo(dToBottomFromResetPos,0);
                    break;
                }

            }
        }
    }

    private boolean isStone(VuforiaTrackable trackable) {
        return ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() && trackable.getName().equals("Stone Target");
    }

    private int zAngle(VuforiaTrackableDefaultListener stone) {
        OpenGLMatrix loc = stone.getUpdatedRobotLocation();
        Orientation rot = Orientation.getOrientation(loc, EXTRINSIC, XYZ, DEGREES);
        return (int)rot.thirdAngle;
    }

    private void swallowStone() {
        wheels.moveTo(0,dToStone);
        while (wheelsBusy()) {
            telemetry.addData("Robob is","approaching skystone");
        }

        /*
        TODO: add ability to swallow stones
         */

        wheels.moveTo(0,-dToStone);
        while (wheelsBusy()) {
            telemetry.addData("Robob is","backing away from skystone");
        }
    }
    private void pukeStone() {
        /*
        TODO: add ability to puke stones
         */
    }

    public void park() {}

    private boolean wheelsBusy() {
        return wheels.motors.get(0).isBusy();
    }
}