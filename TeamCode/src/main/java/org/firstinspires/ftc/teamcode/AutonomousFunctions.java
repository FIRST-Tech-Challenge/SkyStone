package org.firstinspires.ftc.teamcode;


import android.content.Context;

//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AutonomousFunctions extends LinearOpMode {

    private static final double turningPower = 0.75;
    private static final double negativeTurningPower = -0.5;
    public long timeInterval = 500;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    AutonomousFunctions(DcMotor leftMotor, DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    AutonomousFunctions(DcMotor leftMotor, DcMotor rightMotor, long timeInterval) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.timeInterval = timeInterval;
    }

    public void TurnLeft() {
        leftMotor.setPower(-0.5);
        rightMotor.setPower(0.75);
        sleep(2 * timeInterval);
    }

    public void TurnRight() {
        leftMotor.setPower(0.75);
        rightMotor.setPower(-0.5);
        sleep(2 * timeInterval);
    }

    public void MoveForwards(long milliseconds, double power) {
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setPower(power);
        rightMotor.setPower(power);
        sleep(milliseconds);

    }

   /* public void InitialiseDetector(GoldAlignDetector detector, Context context) {

        detector.init(context, CameraViewDisplay.getInstance()); // I
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!
    }

    public double DetermineMovementOffset(GoldAlignDetector detector) {
        double xPos = detector.getXPosition();
        // Return distance relative to robot (camera) center
        return xPos - (detector.downscaleResolution.width / 2);
    }


    */
    @Override
    public void runOpMode() {
    }
}
