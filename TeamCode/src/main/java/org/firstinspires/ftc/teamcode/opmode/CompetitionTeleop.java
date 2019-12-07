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

    public static double angleCorrection = 0.03;
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
                if(superstructure.getCurrentState() == Superstructure.SystemState.MANUAL){
                    elevator.setMotorPowers(gamepad2.left_stick_y);
                }
                if(buttonPad.ifOnceDPadUp()){
                    superstructure.doUpAction();
                } else if(buttonPad.ifOnceDPadDown()){
                    superstructure.doDownAction();
                }
                telemetry.addData("If Button Pressed",buttonPad.ifDPadDown());
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
        elevator.stop();
        intake.stop();
    }

    public void updateTelemetry() {
        Pose2d driveTrainLocation = drive.getPoseEstimate();
        telemetry.addData("Drivetrain X: ", driveTrainLocation.getX());
        telemetry.addData("Drivetrain Y: ", driveTrainLocation.getY());
        telemetry.addData("Drivetrain Heading: ", Math.toDegrees(driveTrainLocation.getHeading()));

        telemetry.addData("Elevator Height: ", elevator.getRelativeHeight());
        telemetry.addData("Superstructure State: ", superstructure.getCurrentState());

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

        //Left Front is Index 0, Left Back is Index 1, Right Front is Index 2, Right Back is Index 3
        List<Double> powerValues = new ArrayList<>();

        powerValues.add(velocityX - velocityY - velocityR - anglePowerCorrection);

        powerValues.add(velocityX + velocityY - velocityR - anglePowerCorrection);

        powerValues.add(velocityX + velocityY + velocityR + anglePowerCorrection);

        powerValues.add(velocityX - velocityY + velocityR + anglePowerCorrection);

        double greatestPower = 0;
        for(Double d : powerValues){
            if(Math.abs(d) > greatestPower){
                greatestPower = Math.abs(d);
            }
        }

        for(int i = 0; i < powerValues.size(); i++){
            powerValues.set(i, powerValues.get(i) / greatestPower);
        }

        drive.setMotorPowers(powerValues.get(0), powerValues.get(1), powerValues.get(3), powerValues.get(2));
    }
}
