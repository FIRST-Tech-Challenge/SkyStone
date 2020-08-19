package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MathFunctions;


/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor topRight, botRight, topLeft, botLeft;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rf", rbName = "rb", lfName = "lf", lbName = "lb";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, 0, 0);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.getX() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.getY() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        topRight = hardwareMap.dcMotor.get(rfName);
        botRight = hardwareMap.dcMotor.get(rbName);
        topLeft = hardwareMap.dcMotor.get(lfName);
        botLeft = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        botLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        botRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
    public void goToPosition(double targetX, double targetY, double power, double desiredOrientation, double allowableDistanceError){
        double distanceToX = targetX - globalPositionUpdate.getX();
        double distanceToY = targetY - globalPositionUpdate.getY();
        double distance = Math.hypot(distanceToX, distanceToY);
        while(opModeIsActive() && distance > allowableDistanceError) {
            distanceToX = targetX - globalPositionUpdate.getX();
            distanceToY = targetY - globalPositionUpdate.getY();
            distance = Math.hypot(distanceToX, distanceToY);
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));
            double ly = calculateY(robotMovementAngle, power);
            double lx = calculateX(robotMovementAngle, power);
            double rx = desiredOrientation - globalPositionUpdate.returnOrientation();
            topLeft.setPower(Range.clip(ly + lx + rx, -power, power));
            topRight.setPower(Range.clip(ly - lx - rx, -power, power));
            botLeft.setPower(Range.clip(ly - lx + rx, -power, power));
            botRight.setPower(Range.clip(ly + lx - rx, -power, power));
        }
    }
    public void goTo(double x, double y, double power){
        double distance = Math.hypot(x - globalPositionUpdate.getX(), y - globalPositionUpdate.getY());
        double absAngleToTarget = Math.atan2(y - Math.toRadians(globalPositionUpdate.returnOrientation()), x - Math.toRadians(globalPositionUpdate.returnOrientation()));
        double relAngleToPoint = MathFunctions.AngleWrap(absAngleToTarget - (Math.toRadians(globalPositionUpdate.returnOrientation())));
        double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
        double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
        double movementXPower = (relativeXToPoint / Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)) * power;
        double movementYPower = (relativeYToPoint / Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)) * power;
        topLeft.setPower(Range.clip(movementYPower + movementXPower , -power, power));
        topRight.setPower(Range.clip(movementYPower - movementXPower, -power, power));
        botLeft.setPower(Range.clip(movementYPower - movementXPower , -power, power));
        botRight.setPower(Range.clip(movementYPower + movementXPower, -power, power));
    }
    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
