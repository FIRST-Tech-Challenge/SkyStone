package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import pkg3939.Robot3939;

/**
 * Created by maryjane on 11/6/2018.
 * after 1st meet
 * mecanum wheels
 */

@TeleOp(name="Holonomic", group="Iterative Opmode")
//@Disabled
public class
Holonomic extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declaration of the motors and servos goes here
    Robot3939 robot = new Robot3939();

    public static final boolean earthIsFlat = true;

    public static final boolean DOWN = true, UP = false;

    @Override //when init is pressed
    public void runOpMode(){

        //Naming, Initialization of the hardware, use this deviceName in the robot controller phone
        //use the name of the object in the code
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);
        robot.initIMU(hardwareMap);
        robot.useEncoders(false);

        waitForStart();
        runtime.reset();

        double speedSet = 5;//robot starts with speed 5 due to 40 ratio motors being op
        boolean forks = false;

        while (opModeIsActive()) {

            //bumpers set speed of robot
            if(gamepad1.right_bumper)
                speedSet += 0.0005;
            else if(gamepad1.left_bumper)
                speedSet -= 0.0005;

            speedSet =  Range.clip(speedSet, 1, 10);//makes sure speed is limited at 10.

            if(!gamepad1.right_bumper && !gamepad1.left_bumper)//makes sure speed does not round every refresh. otherwise, speed won't be able to change
                speedSet = Math.round(speedSet);

            //foundation moving forks

            if(gamepad1.a) {
                forks = !forks;
                sleep(50);
            }

            if(forks) {
                robot.setForks(DOWN);
            }
            else if(earthIsFlat) {
                robot.setForks(UP);
            }

            //robot drive
            speedSet /= 10;

            robot.drive(gamepad1.left_stick_x*speedSet,
                        gamepad1.left_stick_y*speedSet,
                        gamepad1.right_stick_x*speedSet);

            telemetry.addData("Drive", "Holonomic");
            telemetry.addData("Global Heading", robot.getAngle());
            telemetry.addData("speedSet", "%.2f", speedSet);
            telemetry.update();
        }
    }
}
