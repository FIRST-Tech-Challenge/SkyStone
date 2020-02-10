/*This class is used for shortcuts e.g. the setAllMotors method used to set all motors at once instead of having to set all of them individually
 * created by coolPseudonym
 */

package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassisGyro;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledExtender;

public class OrientationTools {

    static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CMS      = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);


    //Declare a variable to hand over the right wanted hardwaremap for the right layout
    private HardwareChassis hwchss;
    private LinearOpMode opMode;
    private HardwareChassisGyro hwgy;

    private final double MINSPEED = 0.15;
    /**
     * Constructor
     * @param hwchss Object that inherits from HardwareChassis
     * @param hardwareMap HardwareMap of OpMode
     * @param opMode Calling op Mode
     */
    public OrientationTools(HardwareChassis hwchss, HardwareMap hardwareMap, LinearOpMode opMode) {
        this.hwchss = hwchss;
        hwgy = new HardwareChassisGyro(hardwareMap);
        hwgy.init(hardwareMap);
        this.opMode = opMode;
    }

    /**
     * Used to set all motors at once
     * @param SpeedFrontLeft Speed of FrontLeftMotor
     * @param SpeedBackLeft Speed of BackLeftMotor
     * @param SpeedBackRight Speed of BackRightMotor
     * @param SpeedFrontRight Speed of FrontRightMotor
     */
    public void setAllMotors(double SpeedFrontLeft, double SpeedBackLeft, double SpeedBackRight,double SpeedFrontRight){
        hwchss.motor_front_left.setPower(SpeedFrontLeft);
        hwchss.motor_front_right.setPower(SpeedFrontRight);
        hwchss.motor_rear_left.setPower(SpeedBackLeft);
        hwchss.motor_rear_right.setPower(SpeedBackRight);
    }

    /**
     * This method lets the robot turn
     * @param speed The speed at which the robot should drive (higher values not recommended, not accurate
     * @param direction direction enum, whether it should turn to the left or to the right (enum added by paul)
     */
    public void turn (double speed, DirectionEnum direction){
        if (direction.equals(DirectionEnum.Right)){
            setAllMotors(speed,-speed,-speed,speed);
        }else if(direction.equals(DirectionEnum.Left)){
            setAllMotors(-speed,speed,speed,-speed);
        }
    }

    /**
     * @// TODO: 19.09.2019  please look at this, at the first glance, I don't know, why we ad 180 -Paul
     * Returns actual position of robot
     * @return position as float
     */
    public float getDegree(){
        Orientation angles;
        angles = hwgy.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle + 180;
    }

    public HardwareChassisGyro getHwgy() {
        return hwgy;
    }

    public void setHwgy(HardwareChassisGyro hwgy) {
        this.hwgy = hwgy;
    }

    /**
     * @// TODO: 19.09.2019  please rework
     * Drive method from Nils and Malte
     * @param degree The amount of degrees the robot should turn. Don't enter negative values
     */
    public void turnToDegreeV4(float degree){
        if(degree<0) {
            //Please don't throw errors. We cannot detect this on the device ~Paul
            throw new Error("negative Wert zum drehen");
        }
        double difference = 2;
        /*mby remove*/degree = 360 - degree;
        float goal = (this.getDegree() + degree)%360;
        while((difference>1 || difference<-1 )  && !opMode.isStopRequested()){
            difference = -1*/*Math.abs*/(goal-this.getDegree());
            this.turn(this.personalTanH(3*difference/200), DirectionEnum.Right);
        }
        this.setAllMotors(0,0,0,0);
    }

    /**
     * Reworked hyperbolic tangens function
     * @param i input
     * @return output of tanh
     */
    private double personalTanH(double i){
        double o = Math.tanh(i);
        if (o <this.MINSPEED && o>0){ o = 0.3;} //0.15
        if(o> -this.MINSPEED && o <0){o = -0.3;} //0.15
        return o;
    }

    public double getDegree360(BNO055IMU imu){
        return 180+imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
    }

    public void driveSidewardTime(long timeInMillis, double power, double smoothness, BNO055IMU imu, OmniWheel wheel, OpMode op){
        double current = this.getDegree360(imu);
        long timeStart = System.currentTimeMillis();
        int msStuckinLoopStart = op.msStuckDetectLoop;
        op.msStuckDetectLoop = 1073741824;
        while (timeStart + timeInMillis > System.currentTimeMillis()) {
            double offset = this.getDegree360(imu) - current;
            wheel.setMotors(0, power, offset / smoothness);
        }
        wheel.setMotors(0,0,0);
        op.msStuckDetectLoop = msStuckinLoopStart;
    }


    public void driveSidewardEncoder(OpMode op,double distanceForward, double distanceSideways, double speed,OmniWheel wheel, double startPos, BNO055IMU imu, double smoothness, double smoothnessAdjust) {
        double offset = this.getDegree360(imu) - startPos;

        while(Math.abs(offset)>2 && opMode.opModeIsActive()){
            offset  = this.getDegree360(imu) - startPos;
            wheel.setMotors(0,0,offset/smoothnessAdjust);
        }
        wheel.setMotors(0,0,0);


        double maxDistance = Math.max(Math.abs(distanceForward), Math.abs(distanceSideways));

        double[] wheelSpeeds = OmniWheel.calculate(WHEEL_DIAMETER_CMS / 2, 38, 24, distanceForward / maxDistance, distanceSideways / maxDistance, 0);

        // Determine new target position
        double[] targets = {
                wheel.robot.motor_front_left.getCurrentPosition() + wheelSpeeds[0] * (COUNTS_PER_CM * maxDistance),
                wheel.robot.motor_front_right.getCurrentPosition() + wheelSpeeds[1] * (COUNTS_PER_CM * maxDistance),
                wheel.robot.motor_rear_left.getCurrentPosition() + wheelSpeeds[2] * (COUNTS_PER_CM * maxDistance),
                wheel.robot.motor_rear_right.getCurrentPosition() + wheelSpeeds[3] * (COUNTS_PER_CM * maxDistance)};
        // And pass to motor controller
        wheel.robot.motor_front_left.setTargetPosition((int) targets[0]);
        wheel.robot.motor_front_right.setTargetPosition((int) targets[1]);
        wheel.robot.motor_rear_left.setTargetPosition((int) targets[2]);
        wheel.robot.motor_rear_right.setTargetPosition((int) targets[3]);


        //DcMotor.RunMode prevMode = wheel.robot.motor_front_left.getMode();

        // Turn On RUN_TO_POSITION
        wheel.robot.motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel.robot.motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel.robot.motor_rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel.robot.motor_rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(wheel.robot.motor_front_right.isBusy() && wheel.robot.motor_front_left.isBusy() && wheel.robot.motor_rear_left.isBusy() && wheel.robot.motor_rear_right.isBusy()) {
            offset  = this.getDegree360(imu) - startPos;
            // reset the timeout time and start motion.
            //wheel.robot.motor_front_left.setPower(wheelSpeeds[0] * speed + offset/smoothness);
            //wheel.robot.motor_front_right.setPower(wheelSpeeds[1] * speed + offset/smoothness);
            //wheel.robot.motor_rear_left.setPower(wheelSpeeds[2] * speed + offset/smoothness);
            //wheel.robot.motor_rear_right.setPower(wheelSpeeds[3] * speed + offset/smoothness);
            op.telemetry.addData("dsfsdf",offset);
            op.telemetry.update();
            wheel.setMotors(0,speed,offset/smoothness);
        }


        wheel.robot.motor_front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.robot.motor_front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.robot.motor_rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.robot.motor_rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(Math.abs(offset)>2 && opMode.opModeIsActive()){
            offset  = this.getDegree360(imu) - startPos;
            wheel.setMotors(0,0,offset/smoothnessAdjust);
        }
        wheel.setMotors(0,0,0);

    }


    public void turnToDegrees(double degrees, double smoothness, OmniWheel wheel, BNO055IMU imu){
        double initPos = this.getDegree360(imu);
        double endPos = (360 + initPos + degrees)%360;
        double offset = this.getDegree360(imu) - endPos;
        while(Math.abs(offset)>3){
            offset = this.getDegree360(imu) - endPos;
            wheel.setMotors(0,0,offset/smoothness);
        }
        wheel.setMotors(0,0,0);
    }


}