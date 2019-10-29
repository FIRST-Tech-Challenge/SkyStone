package pkg3939;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot3939 {
    private DcMotor RL, RR, FL, FR;
    private Servo servoRight, servoLeft;

    private final int encoderTicks = 1120;
    private final double wheelDiameter = 3.85827;//in inches

    public void initMotors(HardwareMap hwmap) {
        RL        = hwmap.dcMotor.get("left_drive");
        RR       = hwmap.dcMotor.get("right_drive");
        FL       = hwmap.dcMotor.get("front_left");
        FR      = hwmap.dcMotor.get("front_right");
        servoRight = hwmap.servo.get("servoRight");
        servoLeft = hwmap.servo.get("servoLeft");

        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void move(boolean opMode, double x, double y, double angle, double power) {
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //y
        double distancePerRotationY = 3.1415 * wheelDiameter; //pi * diameter (inches) = circumference
        double rotationsY = y/distancePerRotationY; //distance / circumference (inches)
        int encoderTargetY = (int)(rotationsY*encoderTicks);

        //x
        double distancePerRotationX = 13.5; //distance per rotations is different than circumference when strafing (inches)
        double rotationsX = x/distancePerRotationX; //distance / circumference (inches)
        int encoderTargetX = (int)(rotationsX*encoderTicks);

        //angle
        double ticksPerRotation = 0;//measure how many ticks for a 360 rotation
        double rotationsA = angle/360;
        int encoderTargetA = (int)(rotationsA*ticksPerRotation);

        if(opMode) {
            RL.setTargetPosition(encoderTargetY-encoderTargetX+encoderTargetA);
            RR.setTargetPosition(encoderTargetY+encoderTargetX-encoderTargetA);
            FL.setTargetPosition(encoderTargetY+encoderTargetX+encoderTargetA);
            FR.setTargetPosition(encoderTargetY-encoderTargetX-encoderTargetA);

            RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            RL.setPower(Math.abs(power));//childproof. must have always positive power
            RR.setPower(Math.abs(power));
            FL.setPower(Math.abs(power));
            FR.setPower(Math.abs(power));

            while(RL.isBusy() || RR.isBusy() || FL.isBusy() || FR.isBusy()) {
                //wait till motor finishes working
                telemetry.addData("Path", "Driving");
                telemetry.update();
            }

            RL.setPower(0);
            RR.setPower(0);
            FL.setPower(0);
            FR.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();

            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
