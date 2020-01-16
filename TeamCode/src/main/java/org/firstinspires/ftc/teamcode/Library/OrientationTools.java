/*This class is used for shortcuts e.g. the setAllMotors method used to set all motors at once instead of having to set all of them individually
 * created by coolPseudonym
 */

package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassisGyro;

public class OrientationTools {

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

    public OrientationTools(HardwareChassis hwchss) {
        this.hwchss = hwchss;
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
    public void turn (double speed, Direction_Enum direction){
        if (direction.equals(Direction_Enum.Right)){
            setAllMotors(speed,-speed,-speed,speed);
        }else if(direction.equals(Direction_Enum.Left)){
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
            this.turn(this.personalTanH(3*difference/200),Direction_Enum.Right);
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

        op.msStuckDetectLoop = msStuckinLoopStart;
    }

    public void driveSidewardEncoder(int[] EncoderValues, double power, double smoothness, BNO055IMU imu, OmniWheel wheel, OpMode op){
        double current = this.getDegree360(imu);
        double offset;
        int msStuckinLoopStart = op.msStuckDetectLoop;
        op.msStuckDetectLoop = 1073741824;
        int[] encValuesStart = new int[4];
        encValuesStart = new int[]{
                              wheel.robot.motor_front_left.getCurrentPosition(),
                              wheel.robot.motor_front_right.getCurrentPosition(),
                              wheel.robot.motor_rear_left.getCurrentPosition(),
                              wheel.robot.motor_rear_right.getCurrentPosition()};
        int[] encValuesCurrent = new int[4];
        for(int[] diffrence,stop,encValuesCurrent = new int[]{
                                                                encValuesStart[0]-wheel.robot.motor_front_left.getCurrentPosition(),
                                                                encValuesStart[0]-wheel.robot.motor_front_right.getCurrentPosition(),
                                                                encValuesStart[0]-wheel.robot.motor_rear_left.getCurrentPosition(),
                                                                encValuesStart[0]-wheel.robot.motor_rear_right.getCurrentPosition()}){
            offset  = this.getDegree360(imu) - current;
            wheel.setMotors(0, power, offset / smoothness);
        }

        op.msStuckDetectLoop = msStuckinLoopStart;
    }


}