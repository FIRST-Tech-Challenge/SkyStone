package org.eastsideprep.eps9884;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.eastsideprep.eps8103.Hardware8103;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Test 8103", group="8103")
public class Test8103 extends LinearOpMode{


        /* Declare OpMode members. */
        Hardware8103 robot = new Hardware8103();

        @Override
        public void runOpMode() {
            robot.init(hardwareMap);
            waitForStart();

           // robot.leftBackMotor.setPower(1.0);

          // sleep(5000);

           //robot.leftBackMotor.setPower(-1.0);

            //sleep(4000);

          robot.rightBackMotor.setPower(0.25);

          sleep(5000);

          robot.rightBackMotor.setPower(-0.25);

            sleep(5000);

            robot.leftFrontMotor.setPower(0.25);

            sleep(5000);

            robot.leftFrontMotor.setPower(-0.25);

            sleep(5000);

            robot.rightFrontMotor.setPower(0.25);

            sleep(5000);

            robot.rightFrontMotor.setPower(-0.25);

            sleep(5000);


            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "started");    //
            telemetry.update();
        }
    }



