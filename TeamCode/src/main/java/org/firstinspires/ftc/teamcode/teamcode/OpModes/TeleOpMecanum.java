//Package statement
package org.firstinspires.ftc.teamcode.teamcode.OpModes;

//import statements

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Outtake;

//TeleOp class identifier
@TeleOp(name="Arcade Drive", group="TeleOp")
public class TeleOpMecanum extends OpMode {

    //Instantiate Variables
    DriveTrain drive = new DriveTrain();
    //Intake intake = new Intake();
    //Outtake outtake = new Outtake();
    //Variables for Arcade Drive
    int motorPos = 0;
    double leftStickY;
    double leftStickX;
    double direction;
    double velocity;
    double speed;
    double speedProp = 1.0;
    boolean halfTrue = false;
    boolean cfmToggle = false;
    double direct = 1.0;
    boolean pastX = false;


    //  Variables for Cruise Foundation Moving (CFM)

    private static final double  massFoundation = 1.905; // Mass in kg
    private static final double massStone = .1882;
    static final double muBlocks = .78;
    static final double muMat = .535;
    double fix = 1.0;
    double tolerance = .05;
    double mass = 0.0;
    double foundationFriction = 0.0;
    double maxCFM_Velocity = 0.0;
    double CFM_AungularVelocity = 0.0;
    double cfm_power = 0.0;

    int numberStackedBlocks = 0;


    //Initializes Method
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.resetEncoders();
        drive.runtime.reset();

        //intake.initIntake(this);
        //outtake.initOuttake(this);

        numberStackedBlocks = 0;
        cfmToggle = false;
    }

    //Main Loop
    @Override
    public void loop() {

            telemetry.addData("Status", "Run Time: " + drive.runtime.toString());
            telemetry.addData("Motor Encoder Position", "Average Ticks:" + motorPos);
            telemetry.addData("Motor Position", "Motor Rotation", + speed);

            //Gets Motor Position By Taking Average From All Motors
            motorPos = (drive.fr.getCurrentPosition() * drive.fl.getCurrentPosition() * drive.bl.getCurrentPosition()
                    * drive.br.getCurrentPosition()) / 4;

            //Arcade Controls
            //Latitudinal Direction
            if (Math.abs(gamepad1.left_stick_y) > .05) {
                leftStickY = gamepad1.left_stick_y;
            } else {
                leftStickY = 0;
            }

            //Longitudinal Direction
            if (Math.abs(gamepad1.left_stick_x) > .05) {
                leftStickX = gamepad1.left_stick_x;
            }
            else {
                leftStickX = 0;
            }

            //Speed Reducer
            if (gamepad1.x != pastX) {
                pastX = gamepad1.x;
                if (gamepad1.x) {
                    if (speedProp == 1) {
                        speedProp = 0.5;
                    } else {
                        speedProp = 1;
                    }
                }
            }

        telemetry.addData("Velocity : ", velocity);

        telemetry.addData("Direction : ", direction);

        telemetry.addData("Speed : ", speed);


        //Gets Magnitude of Left Stick
        velocity = Math.hypot(leftStickX, leftStickY);
        //Gets Direction of Left Stick
        direction = Math.atan2(leftStickY, -leftStickX) - Math.PI / 4;
        speed = gamepad1.right_stick_x;

            if (Math.abs(gamepad1.right_stick_x) < 0.075 ) {
                speed = 0;
            }

            if (Math.abs(gamepad1.left_stick_x) < 0.075 && Math.abs(gamepad1.left_stick_y) < 0.075) {
                velocity = 0;
            }

            drive.fl.setPower((velocity * Math.cos(direction) + speed) * speedProp);
            drive.fr.setPower((velocity * Math.sin(direction) - speed) * speedProp);
            drive.bl.setPower((velocity * Math.sin(direction) + speed) * speedProp);
            drive.br.setPower((velocity * Math.cos(direction) - speed) * speedProp);

            //Foundation Moving Toggle
            //Toggle sets speed such that the robot can move the fastest
            //while moving the foundation and not dropping any blocks
            //Takes into account the mass of the foundation and block stack
            //and the friction of the floor

        /*if (gamepad2.dpad_up) {
            while(gamepad1.dpad_down)
            {
            }
            numberStackedBlocks++;
        }
        else if (gamepad2.dpad_down) {
            while(gamepad1.dpad_down)
            {
            }
            numberStackedBlocks--;
        }
            //  Mass of Whole Object
            mass = massFoundation + numberStackedBlocks * massStone;

            //  Max CFM velocity, calculated
            maxCFM_Velocity = fix * Math.sqrt((2 * tolerance * 9.81 * massStone * numberStackedBlocks * muBlocks)
                    / mass);

            //  CFM velocity to Aungular Velocity
            CFM_AungularVelocity = maxCFM_Velocity / (DriveTrain.wheelDiam / 2);

            //  Power to set motors to follow CFM velocity.
            cfm_power = (-1) * (DriveTrain.stallTorque / DriveTrain.noLoadSpeed) * CFM_AungularVelocity
                    + DriveTrain.stallTorque * CFM_AungularVelocity;

            telemetry.addData("Number of Blocks : ", numberStackedBlocks);

            //set up power conversion
            //set up toggle

            if(gamepad1.b && !cfmToggle)
            {
                cfmToggle = true;
            }
            else if(gamepad1.b && cfmToggle)
            {
                cfmToggle = false;
            }

            telemetry.addData("CFM Toggle : ", cfmToggle);

            //Gets Magnitude of Left Stick
            velocity = Math.hypot(leftStickX, leftStickY);
            //Gets Direction of Left Stick
            direction = Math.atan2(leftStickY, -leftStickX) - Math.PI / 4;
            speed = gamepad1.right_stick_x;

            //Sets Power to Wheel
            if(!cfmToggle)
            {*/

         /*   }
            else if(cfmToggle)
            {

                direct = Math.abs(gamepad1.left_stick_x)/gamepad1.left_stick_x;
                // setPower(cfm_power)
                drive.fl.setPower(cfm_power * direct);
                drive.fr.setPower(-cfm_power * direct);
                drive.bl.setPower(-cfm_power * direct);
                drive.br.setPower(cfm_power * direct);
            }

            telemetry.addData("CFM Power : ", cfm_power);*/


            // dont need, implemented in outtake class already
            /*
            if (gamepad2.a && outtake.pushBlock.getPosition() != .5) {
                outtake.pushBlock.setPosition(.5);
            } else if (gamepad2.a && outtake.pushBlock.getPosition() == .5) {
                outtake.pushBlock.setPosition(1);
            }
             */

            //Intake
          //  intake.compliantIntake_TeleOp();

            //Outtake
        //   outtake.outTake_TeleOp();

            telemetry.update();
    }
}
