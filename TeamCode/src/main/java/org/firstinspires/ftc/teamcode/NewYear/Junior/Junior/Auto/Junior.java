//package org.firstinspires.ftc.teamcode.NewYear.Junior;
//
//import android.os.SystemClock;
//import android.util.Log;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.CstArray;
//
//import java.lang.reflect.Array;
//import java.util.ArrayList;
//import java.util.Dictionary;
//import java.util.List;
//
//public class Junior {
//
//    //drive motors
//    public DcMotor leftDrive;
//    public DcMotor rightDrive;
//
//    //imu
//    private BNO055IMU imu;
//    private Orientation angles;
//    private Position position;
//
//    //inherited classes from op mode
//    private Telemetry telemetry;
//    private HardwareMap hardwareMap;
//    private LinearOpMode linearOpMode;
//
//    public Junior (HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode){
//        this.telemetry = telemetry;
//        this.hardwareMap = hardwareMap;
//        this.linearOpMode = linearOpMode;
//
//        //config names need to match configs on the phone
//        //Map drive motors
//        leftDrive = hardwareMap.dcMotor.get("left_drive");
//        rightDrive = hardwareMap.dcMotor.get("right_drive");
//
//        //Map LinearSlide Motors
//        //Set direction of drive motors
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//
//    }
//
//    public void intializeIMU() {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
//    }
//
//    public void resetEncoders() {
//        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    public void changeRunModeToUsingEncoder(){
//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void setMotorMode(DcMotor.RunMode runMode){
//        leftDrive.setMode(runMode);
//        rightDrive.setMode(runMode);
//    }
//
//    public void timeMove() {
//        long startTime = SystemClock.elapsedRealtime();
//
//        leftDrive.setPower(1);
//        rightDrive.setPower(1);
//
//        linearOpMode.sleep(5000);
//        leftDrive.setPower(0);
//        rightDrive.setPower(0);
//    }
//
//    public void driveMotorsBreakZeroBehavior() {
//        //sets drive motors to brake mode
//        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void brakeRobot() {
//        //brakes robot
//        driveMotorsBreakZeroBehavior();
//        leftDrive.setPower(0);
//        rightDrive.setPower(0);
//    }
//
//    public void turn (double targetHeading){
//        targetHeading  = -1 * targetHeading;
//        targetHeading = Range.clip(targetHeading, -179,179);
//        double power;
//        long startTime = SystemClock.elapsedRealtime();
//
//        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double startHeading = angles.firstAngle;
//        double maxAngle = Math.abs(startHeading - targetHeading);
//
//        if (maxAngle < 1.0){
//            return;
//        }
//
//        while(linearOpMode.opModeIsActive()){
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            power = (targetHeading - angles.firstAngle) / maxAngle;
//            if(Math.abs(power) < 0.01){
//                if (power > 0){
//                    power = 0.01;
//                } else if (power < 0){
//                    power = -0.01;
//                } else {
//                    break;
//                }
//            }
//            if (Math.abs(angles.firstAngle - startHeading) > maxAngle || ((SystemClock.elapsedRealtime() - startTime) > 2000)){
//                break;
//            }
//            leftDrive.setPower(-power);
//            rightDrive.setPower(power);
//        }
//        brakeRobot();
//        linearOpMode.sleep(100);
//        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    public void splineCalculation(double[][] waypoints){
//
//        final double distWheels = 10; //inches
//        int numdata = waypoints[0].length;
//        double Nu = 100;
//        double num = 0;
//
//        ArrayList<Double> xV1 = new ArrayList<>();
//        ArrayList<Double> yV1 = new ArrayList<>();
//
//        ArrayList<Double> distance = new ArrayList<>();
//
//        ArrayList<Double> uV = new ArrayList<>();
//
//        ArrayList<Double> xV = new ArrayList<>();
//        ArrayList<Double> yV = new ArrayList<>();
//        ArrayList<Double> dxdu = new ArrayList<>();
//        ArrayList<Double> dydu = new ArrayList<>();
//
//        ArrayList<Double> dxdu1 = new ArrayList<>();
//        ArrayList<Double> dydu1 = new ArrayList<>();
//        double[][] leftWheel = new double[uV.size()][2];
//        double[][] rightWheel = new double[uV.size()][2];
//
//        ArrayList<Double> u = new ArrayList<>();
//        int j =0;
//
//        while(num != 1-1/Nu){
//            num = j/Nu;
//            u.add(num);
//            j++;
//        }
//        for(int i = 1; i<numdata; i++){
//
//        }
//
//        xV.add(waypoints[waypoints.length-1][1]);
//
//        yV.add(waypoints[waypoints.length][2]);
//        distance.add(0.0);
//        for(int i = 0;i<xV.size();i++){
//            for(int k = 0;k<u.size();k++){
//                double uElement = u.get(k);
//                xV1.add((Math.pow(uElement,3)*2-3 * Math.pow(uElement,2)+1) * waypoints[i][1] + (-2 + Math.pow(uElement,3) + 3 * Math.pow(uElement,2)) * waypoints[i+1][1] + (Math.pow(uElement,3)-2 * Math.pow(uElement,2) + uElement) * waypoints[i][3] + (Math.pow(uElement,3)-Math.pow(uElement,2)) * waypoints[i+1][3]);
//                yV1.add((Math.pow(uElement,3)*2-3 * Math.pow(uElement,2)+1) * waypoints[i][2] + (-2 + Math.pow(uElement,3) + 3 * Math.pow(uElement,2)) * waypoints[i+1][2] + (Math.pow(uElement,3)-2 * Math.pow(uElement,2) + uElement) * waypoints[i][4] + (Math.pow(uElement,3)-Math.pow(uElement,2)) * waypoints[i+1][4]);
//
//                dxdu1.add((6* Math.pow(uElement,2)-6*uElement) * (waypoints[i][1]-waypoints[i+1][1]) + waypoints[i][3] * (3 * Math.pow(uElement,2)-4 * uElement +1) + waypoints[i+1][3] * (3* Math.pow(uElement,2) -2 * uElement));
//                dydu1.add((6* Math.pow(uElement,2)-6*uElement) * (waypoints[i][2]-waypoints[i+1][2]) + waypoints[i][4] * (3 * Math.pow(uElement,2)-4 * uElement +1) + waypoints[i+1][4] * (3* Math.pow(uElement,2) -2 * uElement));
//
//                uV.add((i-1)*Nu+1);
//
//
//            }
//
//            for()
//        }
//
//    }
//
//    public void splineMove(double[][] data, double maxSpeed) {
//
//        final String TAG = "splineMove";
//
//        resetEncoders();
//        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        long startTime = SystemClock.elapsedRealtime();
//        double refLeftSpeed, refRightSpeed;
//        double refLeftDistance, refRightDistance;
//        double refHeading;
//        double leftPower, rightPower;
//        double leftDistance, rightDistance;
//        double heading;
//        int inc;
//        int i;
//        double encoderToInches = 60/5150;  //5150 encoders = 60 inches
//        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double startHeading = angles.firstAngle;
//
//        //create file
//
//        /*double maxSpeed = 0;
//        for (i=0; i<data.length; i++){
//            if (maxSpeed < data[i][0]){
//                maxSpeed = data[i][0];
//            }
//            if (maxSpeed < data[i][1]){
//                maxSpeed = data[i][1];
//            }
//        }
//        */
//
//        while (linearOpMode.opModeIsActive()){
//
//            double dt = SystemClock.elapsedRealtime() - startTime; //in milli
//            dt = dt / 1000; //in seconds
//
//            if (dt < data[data.length - 1][2]){                //find increment at any time to do interpolation
//                inc = -1;
//                for (i=0; i<data.length - 2; i++){
//                    if (data[i][2] <= dt && dt < data[i+1][2]){
//                        inc = i;
//                        break;
//                    }
//                }
//                if (inc < 0){
//                    brakeRobot();
//                    break;
//                }
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                // find the left and right speed by interpolation from data file
//                refLeftSpeed  = ((data[inc+1][0] - data[inc][0]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][0];
//                refRightSpeed = ((data[inc+1][1] - data[inc][1]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][1];
//
//                // find the left and right distance by interpolation from data file
//                refLeftDistance = ((data[inc+1][4] - data[inc][4]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];
//                refRightDistance = ((data[inc+1][5] - data[inc][5]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];
//
//                // find the heading by interpolation from data file
//                refHeading =((data[inc+1][6] - data[inc][6]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][6] + startHeading;
//
//                // find the left and right encoder values and convert them to distance traveled in inches
//                leftDistance = leftDrive.getCurrentPosition() * encoderToInches;
//                rightDistance = rightDrive.getCurrentPosition() * encoderToInches;
//
//                // find the heading of robot
//                heading = angles.firstAngle;
//
//                // old algorithm
//                //leftPower = refLeftSpeed/maxSpeed;
//                //rightPower = refRightSpeed/maxSpeed;
//
//                // find power
//                leftPower =  refLeftSpeed /maxSpeed - (refHeading - heading) / 30;   //(refLeftDistance  - leftDistance ) / 1000
//                rightPower = refRightSpeed/maxSpeed  + (refHeading - heading) / 30;  //+ (refRightDistance - rightDistance) / 1000
//
//                // set power
//                leftDrive.setPower(leftPower);
//                rightDrive.setPower(rightPower);
//
//                //write to file
//                telemetry.addLine(dt+ " - LeftDistance: " + (refLeftDistance-leftDistance) + " RightDistance: " + (refRightDistance-rightDistance) + " leftPower: " + (refLeftSpeed-leftPower) + " rightPower: " + (refRightSpeed-rightPower) + " heading: " + (refHeading - heading));
//                telemetry.addLine();
//                Log.i(TAG, "Time: " + dt + " leftError: " + (refLeftDistance-leftDistance) + " rightError: " + (refRightDistance-rightDistance) + " leftPower: " + (refLeftSpeed-leftPower) + " rightPower: " + (refRightSpeed-rightPower) + " heading: " + (refHeading-heading));
//
//            } else {
//                brakeRobot();
//                break;
//            }
//        }
//
//    }
//}
