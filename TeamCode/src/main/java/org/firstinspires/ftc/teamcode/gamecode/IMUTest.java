//package org.firstinspires.ftc.teamcode.gamecode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
//import org.firstinspires.ftc.teamcode.robots.Joules;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.util.ElapsedTime;​
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
////import org.firstinspires.ftc.robotcore.external.Func;​
//import java.util.Locale;
//
//
//@Autonomous
//public class IMUTest extends AutoOpMode {
//
//    public void runOp() throws InterruptedException {
//        Joules joules = new Joules();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//
//        telemetry.addData("Status", "initialized");
//
//        waitForStart();
//
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        telemetry.addData("angle1", angles.firstAngle);
//        telemetry.addData("angle2", angles.secondAngle);
//        telemetry.addData("angle3", angles.thirdAngle);
//
//
//
//        public void DriveForwardsIMU(double speed){
//            //as of now, current angle is the one that changes, inital angle is the one we start with first
//            double anglecorrection = 0;
//            int pconstant = 0; //proportinal constant if im right+
//            double initalangle = currentangle();//need to change this when i figure out which is the one that changes
//            double anglediffernce = Math.round(initalangle - currentangle); //is this the right round? do i need to round?? who knows
//
//            if (anglediffernce != 0) {
//                anglecorrection = anglediffernce * pconstant;
//            } else {
//                anglecorrection = 0;
//            }
//            // angle correction is posivite when it needs to turn right
//            //angle correction is half power to each wheel
//
//
//            //consider to get the robot to strafe instead of turn ?
//            joules.FrontLeft.setPower(-speed - anglecorrection);
//            joules.FrontRight.setPower(speed + anglecorrection);
//            joules.BackLeft.setPower(-speed - anglecorrection);
//            joules.BackRight.setPower(speed + anglecorrection);
//
//        }
//
//
//        }
//
//}
