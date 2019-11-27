package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.StructureConstructor;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.OneByOneByFive;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.TwoByTwoByFive;
import org.firstinspires.ftc.teamcode.hardware.Elevator;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Superstructure;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Competition Teleop")
public class CompetitionTeleop extends LinearOpMode {

    private SampleMecanumDriveREVOptimized drive;
    private Elevator elevator;
    private Intake intake;
    private FoundationGrabber foundationGrabber;
    private Superstructure superstructure;

    public static double angleCorrection = 0.05;
    private double startingAngle = 0;

    //If the boolean below is false, then it will attempt to store a new angle for correction. If it is true, then the robot is translating and is referencing the previous angle.
    private boolean ifStartingAngle = false;


    @Override
    public void runOpMode() {

        drive = SampleMecanumDriveREVOptimized.getInstance(hardwareMap);
        elevator = Elevator.getInstance(hardwareMap);
        intake = Intake.getInstance(hardwareMap);
        foundationGrabber = FoundationGrabber.getInstance(hardwareMap);

        OmegaGamepad buttonPad = new OmegaGamepad(gamepad2);

        superstructure = new Superstructure(elevator, intake);

        waitForStart();
        intake.release();
        while (!isStopRequested()) {

            //Foundation Grabber
            if (gamepad1.a) {
                foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT);
            } else {
                foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.UP_LEFT);
            }

            //Intake Control
            if (gamepad2.a) {
                intake.setGrabbing();
                superstructure.setManual();
            }
            if (gamepad2.b) {
                intake.open();
                superstructure.setManual();
            }

            //Elevator Control
            if(gamepad2.left_stick_y > 0.05 || gamepad2.left_stick_y < -0.05){
                superstructure.setManual();
                elevator.setMotorPowers(gamepad2.left_stick_y);
            } else {
                if(buttonPad.ifOnceDPadUp()){
                    superstructure.doUpAction();
                } else if(buttonPad.ifOnceDPadDown()){
                    superstructure.doDownAction();
                }
            }
            
            //Drive Control
            if ((gamepad1.left_stick_y < 0.05 && gamepad1.left_stick_y > -0.05) || (gamepad1.left_stick_x < 0.05 && gamepad1.left_stick_x > -0.05) || (gamepad1.right_stick_x < 0.05 && gamepad1.right_stick_x > -0.05)) {
                setTeleopPower(new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        (-gamepad1.right_stick_x) / 1.5)
                );
            } else {
                drive.setDrivePower(new Pose2d(0, 0, 0));
            }

            updateTelemetry();

            buttonPad.update();
            superstructure.update();
        }
    }

    public void updateTelemetry() {
        Pose2d driveTrainLocation = drive.getPoseEstimate();
        telemetry.addData("Drivetrain X: ", driveTrainLocation.getX());
        telemetry.addData("Drivetrain Y: ", driveTrainLocation.getY());
        telemetry.addData("Drivetrain Heading: ", Math.toDegrees(driveTrainLocation.getHeading()));

        telemetry.addData("Elevator Height: ", elevator.getRelativeHeight());

        telemetry.update();
    }

    public void setTeleopPower(Pose2d pose) {

        Double velocityX = pose.getX();
        Double velocityY = pose.getY();
        Double velocityR = pose.getHeading();

        double anglePowerCorrection = 0;
        if (velocityR < 0.05 && velocityR > -0.05) {
            if (!ifStartingAngle) {
                ifStartingAngle = true;
                startingAngle = drive.getPoseEstimate().getHeading();
            }
            anglePowerCorrection = angleCorrection * (startingAngle - drive.getPoseEstimate().getHeading());
        } else if ((velocityR >= 0.05 || velocityR <= -0.05) || ((velocityX <= 0.05 && velocityX >= -0.05) && (velocityY <= 0.05 && velocityY >= -0.05))) {
            ifStartingAngle = false;
        }

        List<Double> translationValues = new ArrayList<>();
        //Front Left
        translationValues.add(velocityX + velocityY);

        //Front Right
        translationValues.add(velocityX - velocityY);

        //Back Left
        translationValues.add(velocityX - velocityY);

        //Back Right
        translationValues.add(velocityX + velocityY);

        List<Double> rotationValues = new ArrayList<>();
        //Front Left
        rotationValues.add(velocityR + anglePowerCorrection);

        //Front Right
        rotationValues.add(-velocityR - anglePowerCorrection);

        //Back Left
        rotationValues.add(velocityR + anglePowerCorrection);

        //Back Right
        rotationValues.add(-velocityR - anglePowerCorrection);

        double scaleFactor = 1;
        double tmpScale = 1;

        for (int i = 0; i < 4; i++) {
            if (Math.abs(translationValues.get(i) + rotationValues.get(i)) > 1) {
                tmpScale = (1 - rotationValues.get(i)) / translationValues.get(i);
            } else if (translationValues.get(i) + rotationValues.get(i) < -1) {
                tmpScale = (rotationValues.get(i) - 1) / translationValues.get(i);
            }
            if (tmpScale < scaleFactor) {
                scaleFactor = tmpScale;
            }
        }


        List<Double> valuesScaled = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            valuesScaled.add(translationValues.get(i) * scaleFactor + rotationValues.get(i));
        }

        drive.setMotorPowers(valuesScaled.get(0), valuesScaled.get(2), valuesScaled.get(3), valuesScaled.get(1));
    }
}
