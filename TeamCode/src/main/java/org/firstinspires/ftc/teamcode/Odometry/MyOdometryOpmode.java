package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Coordinate;
import org.firstinspires.ftc.teamcode.CurvePoint;
import org.firstinspires.ftc.teamcode.MathFunctions;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.MathFunctions.*;


/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor topRight, botRight, topLeft, botLeft;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 800;
            //307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fr", rbName = "br", lfName = "fl", lbName = "bl";
    String verticalLeftEncoderName = "iL", verticalRightEncoderName = rbName, horizontalEncoderName = "iR";

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        ArrayList<CurvePoint> allPoints = new ArrayList<CurvePoint>();
        allPoints.add(new CurvePoint(0,0,0.3,0.3,50,Math.toRadians(50), 1));
        allPoints.add(new CurvePoint(20 * COUNTS_PER_INCH,48 * COUNTS_PER_INCH,0.3,0.3,50,Math.toRadians(50), 1));
        //allPoints.add(new CurvePoint(24 * COUNTS_PER_INCH,48 * COUNTS_PER_INCH,0.2,0.3,50,Math.toRadians(50), 1));
        //allPoints.add(new CurvePoint(0,0,0.2,0.3,50,Math.toRadians(50), 1));
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, 0, 0);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            goTo(20*COUNTS_PER_INCH, 20*COUNTS_PER_INCH, 0.3, 0 , 0.3);
            //followCurve(allPoints, Math.toRadians(90));
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.getX()/ COUNTS_PER_INCH);
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
//    public void goToPosition(double targetX, double targetY, double power, double desiredOrientation, double allowableDistanceError){
//        double distanceToX = targetX - globalPositionUpdate.getX();
//        double distanceToY = targetY - globalPositionUpdate.getY();
//        double distance = Math.hypot(distanceToX, distanceToY);
//        while(opModeIsActive() && distance > allowableDistanceError) {
//            distanceToX = targetX - globalPositionUpdate.getX();
//            distanceToY = targetY - globalPositionUpdate.getY();
//            distance = Math.hypot(distanceToX, distanceToY);
//            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));
//            double ly = calculateY(robotMovementAngle, power);
//            double lx = calculateX(robotMovementAngle, power);
//            double rx = desiredOrientation - globalPositionUpdate.returnOrientation();
//            topLeft.setPower(Range.clip(ly + lx + rx, -power, power));
//            topRight.setPower(Range.clip(ly - lx - rx, -power, power));
//            botLeft.setPower(Range.clip(ly - lx + rx, -power, power));
//            botRight.setPower(Range.clip(ly + lx - rx, -power, power));
//        }
//    }
    public void goTo(double x, double y, double power, double preferredAngle, double turnSpeed){
        double distance = Math.hypot(x - globalPositionUpdate.getX(), y - globalPositionUpdate.getY());
        double absAngleToTarget = Math.atan2(y - Math.toRadians(globalPositionUpdate.returnOrientation()), x - Math.toRadians(globalPositionUpdate.returnOrientation()));
        double relAngleToPoint = MathFunctions.AngleWrap(absAngleToTarget - (Math.toRadians(globalPositionUpdate.returnOrientation())));
        double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
        double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
        double movementXPower = (relativeXToPoint / Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)) * power;
        double movementYPower = (relativeYToPoint / Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)) * power;
        double relativeTurnAngle = relAngleToPoint - Math.toRadians(180) + preferredAngle;
        double movementTurn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1)*turnSpeed;
        topLeft.setPower(Range.clip(movementYPower + movementXPower + movementTurn, -power, power));
        topRight.setPower(Range.clip(movementYPower - movementXPower - movementTurn, -power, power));
        botLeft.setPower(Range.clip(movementYPower - movementXPower + movementTurn, -power, power));
        botRight.setPower(Range.clip(movementYPower + movementXPower - movementTurn, -power, power));
    }
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed){
        double distance = Math.hypot(pt.getX() - globalPositionUpdate.getX(), pt.getY() - globalPositionUpdate.getY());
        double absAngleToTarget = Math.atan2(pt.getY() - Math.toRadians(globalPositionUpdate.returnOrientation()), pt.getX() - Math.toRadians(globalPositionUpdate.returnOrientation()));
        double relAngleToPoint = MathFunctions.AngleWrap(absAngleToTarget - (Math.toRadians(globalPositionUpdate.returnOrientation())));
        double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
        double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
        double movementXPower = (relativeXToPoint / Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)) * power;
        double movementYPower = (relativeYToPoint / Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)) * power;
        double relativeTurnAngle = relAngleToPoint - Math.toRadians(180) + preferredAngle;
        double movementTurn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1)*turnSpeed;
        topLeft.setPower(Range.clip(movementYPower + movementXPower + movementTurn, -power, power));
        topRight.setPower(Range.clip(movementYPower - movementXPower - movementTurn, -power, power));
        botLeft.setPower(Range.clip(movementYPower - movementXPower + movementTurn, -power, power));
        botRight.setPower(Range.clip(movementYPower + movementXPower - movementTurn, -power, power));
    }
    public  void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints, globalPositionUpdate, allPoints.get(0).followDistance);
        goTo(followMe, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }
    public  CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Coordinate location, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for(int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Coordinate> intersections =  lineCircleIntersection(location, followRadius, startLine.toPoint(), endLine.toPoint());
            double closestAngle = 100000000;
            for(Coordinate thisIntersection : intersections){
                //double ang = globalPositionUpdate.angleTo(thisIntersection, true);
                double angle = Math.atan2(thisIntersection.getY() - globalPositionUpdate.getX(), thisIntersection.getY() - globalPositionUpdate.getY());
                double deltaAngle = Math.abs(AngleWrap(angle - globalPositionUpdate.returnOrientation()));
                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
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
