package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Sensors;


@TeleOp(name="TrollMac", group= "Troll")
public class TeleOpTrollTest extends OpMode {

    DriveTrain drive = new DriveTrain();
    Sensors sensors = new Sensors();

    //Instantiate Variables

    //Variables for Arcade Drive
    int motorPos = 0;
    double leftStickY;
    double leftStickX;
    double direction;
    double velocity;
    double speed;
    double speedProp = 1.0;
    boolean pastX = false;
    // boolean cfmToggle = false;
    // double direct = 1.0;
    boolean pastDPadUp = false;
    boolean pastDPadDown = false;

    double flMod = 0;
    double frMod = 0;
    double frHolo = 1;
    double flHolo = 1;
    double brHolo = 1;
    double blHolo = 1;
    double radiax;
    boolean radiaxSet = false;

    //  Variables for Cruise Foundation Moving (CFM)
    /*
    ElapsedTime cfmTime = new ElapsedTime();

    private static final double massFoundation = 1.905; // Mass in kg
    private static final double massStone = .1882;
    static final double muBlocks = .78;
    static final double muMat = .535;
    double fix = 1.0;
    double tolerance = .05;
    double mass = 0.0;
    double foundationFriction = 0.0;
    double maxCFM_Velocity = 0.0;
    double maxCFM_Acceleration = 0.0;
    double CFM_AungularVelocity = 0.0;
    double CFM_Velocity = 0.0;
    double cfm_power = 0.0;

    int numberStackedBlocks = 0;

    */
    //Holon Variables

    double frHolon = 0.0;
    double flHolon = 0.0;
    double brHolon = 0.0;
    double blHolon = 0.0;

    //Initializes Method
    @Override
    public void init() {

        //cfmToggle = false;

        sensors.initSensors(this);

        //Sets Hardware Map
        drive.fl = hardwareMap.dcMotor.get("fl");
        drive.fr = hardwareMap.dcMotor.get("fr");
        drive.bl = hardwareMap.dcMotor.get("bl");
        drive.br = hardwareMap.dcMotor.get("br");

        //Sets Motor Directions
        drive.fl.setDirection(DcMotor.Direction.REVERSE);
        drive.fr.setDirection(DcMotor.Direction.FORWARD);
        drive.bl.setDirection(DcMotor.Direction.REVERSE);
        drive.br.setDirection(DcMotor.Direction.FORWARD);

        //Set Power For Static Motors - When Robot Not Moving
        drive.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.runtime.reset();

        //numberStackedBlocks = 0;

    }

    //Main Loop
    @Override
    public void loop() {


        // motorPos = (drive.fr.getCurrentPosition() * drive.fl.getCurrentPosition() * drive.bl.getCurrentPosition()
        // * drive.br.getCurrentPosition()) / 4;
        speed = gamepad1.right_stick_x;

        telemetry.addData("Status", "Run Time: " + drive.runtime.toString());
        //  telemetry.addData("Motor Encoder Position", "Average Ticks:" + motorPos);
        telemetry.addData("Motor Position", "Motor Rotation", +speed);

        //Gets Motor Position By Taking Average From All Motors


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
        } else {
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


        //Foundation Moving Toggle
        //Toggle sets speed such that the robot can move the fastest
        //while moving the foundation and not dropping any blocks
        //Takes into account the mass of the foundation and block stack
        //and the friction of the floor

        //  Counter Assumes Each Layer is 2 blocks
        /*
        if (gamepad2.dpad_up != pastDPadUp) {
            pastDPadUp = gamepad2.dpad_up;
            if (gamepad2.dpad_up) {
                numberStackedBlocks += 2;
            }
        } else if (gamepad2.dpad_down != pastDPadDown) {
            pastDPadDown = gamepad2.dpad_down;
            if (gamepad2.dpad_down) {
                numberStackedBlocks -= 2;
            }
        }

        //  Mass of Whole Object
        mass = massFoundation + numberStackedBlocks * massStone;

        //  Max CFM Acceleration, calculated

        maxCFM_Acceleration = 9.81 * muBlocks * massStone * numberStackedBlocks / mass;
        */

        //telemetry.addData("Number of Blocks : ", numberStackedBlocks);


        //set up power conversion
        //set up toggle


        telemetry.addData("Velocity : ", velocity);

        telemetry.addData("Direction : ", direction);

        telemetry.addData("Speed : ", speed);

        telemetry.addData("Right HoloMod: ", frHolo);

        telemetry.addData("Left HoloMod: ", flHolo);

        //Sets Power to Wheel
        /*
        if (gamepad1.b && !cfmToggle) {
            cfmToggle = true;
        } else if (gamepad1.b && cfmToggle) {
            cfmToggle = false;
        }

        telemetry.addData("CFM Toggle : ", cfmToggle);
*/
        //Gets Magnitude of Left Stick
        if (!gamepad1.dpad_left) {
            velocity = Math.hypot(leftStickX, leftStickY);
            //Gets Direction of Left Stick
            direction = Math.atan2(leftStickY, -leftStickX) - Math.PI / 4;
            speed = gamepad1.right_stick_x;

            if (Math.abs(gamepad1.right_stick_x) < 0.075) {
                speed = 0;
            }

            if (Math.abs(gamepad1.left_stick_x) < 0.075 && Math.abs(gamepad1.left_stick_y) < 0.075) {
                velocity = 0;
            }

            if (Math.abs(gamepad1.right_trigger) > 0.5f) {
                drive.holonize(drive.fl, drive.bl, flHolo, blHolo, false);
                drive.holonize(drive.fr, drive.br, frHolo, brHolo, true);
            } else {
                brHolo = 1;
                frHolo = 1;
                flHolo = 1;
                blHolo = 1;
            }


            //NEED TO RE-ADD SPEED PROP

            drive.fl.setPower((velocity * Math.cos(direction) + speed) * flHolo);
            drive.fr.setPower((velocity * Math.sin(direction) - speed) * frHolo);
            drive.bl.setPower((velocity * Math.sin(direction) + speed) * blHolo);
            drive.br.setPower((velocity * Math.cos(direction) - speed) * brHolo);
        }
        else if (gamepad1.dpad_left) {
            drive.fl.setPower(1);
            drive.bl.setPower(-1);
            drive.fr.setPower(-1);
            drive.br.setPower(1);
        }

        if (gamepad1.a) {
            telemetry.addData("Left Front: ", drive.fl.getCurrentPosition());
            telemetry.addData("Right Front: ", drive.fr.getCurrentPosition());
            telemetry.addData("Right Back: ", drive.br.getCurrentPosition());
            telemetry.addData("Left Back: ", drive.bl.getCurrentPosition());
            telemetry.update();
        }

            /* if (frHolo * frHolon != flHolo * flHolon) {
                if (frHolo > flHolo) {
                    frHolo = flHolo/frHolo;
                    frHolo = (flHolon * flHolo) / frHolon;
                } else {
                    frHolo = frHolo/flHolo;
                    frHolo = (flHolon * flHolo) / frHolon;
                }
            }

           */

        //}
        //else if (false) {
        //  Max CFM velocity, calculated
            /*
            maxCFM_Velocity = fix * Math.sqrt((2 * tolerance * 9.81 * massStone * numberStackedBlocks * muBlocks)
                    / mass);

            if (CFM_Velocity <= maxCFM_Velocity) {
                CFM_Velocity = maxCFM_Velocity * cfmTime.seconds();
            }

            //  CFM velocity to Aungular Velocitys
            CFM_AungularVelocity = CFM_Velocity / (DriveTrain.wheelDiam / 2);

            //  Power to set motors to follow CFM velocity.
            cfm_power = (-1) * (DriveTrain.stallTorque / DriveTrain.noLoadSpeed) * CFM_AungularVelocity
                    + DriveTrain.stallTorque * CFM_AungularVelocity;


            if (gamepad1.left_stick_x > 0.5) {
                direct = 1;
            } else if (gamepad1.left_stick_x < -0.5) {
                direct = -1;
            } else {
                direct = 0;
                cfmTime.reset();
                CFM_Velocity = 0;
            }


            drive.fl.setPower(-cfm_power * direct + flMod);
            drive.fr.setPower(cfm_power * direct + frMod);
            drive.bl.setPower(cfm_power * direct);
            drive.br.setPower(-cfm_power * direct);

            if(flHolon > brHolon + 0.25 ||
                flHolon < brHolon - 0.25) {
                if (flHolon > brHolon + 0.25) {
                    flMod = flMod - 0.25;
                }
                else {
                    flMod = flMod + 0.25;
                }
            }


            if (frHolon > blHolon + 0.25 ||
            frHolon < blHolon - 0.25) {
                if (frHolon > blHolon + 0.25) {
                    frMod = frMod - 0.25;
                }
                else {
                    frMod = frMod + 0.25;
                }
            }
             */
        //   }

        // telemetry.addData("CFM Power : ", cfm_power);
/*
            if (gamepad1.dpad_left) {
                drive.fl.setPower(1);
                drive.fr.setPower(-1);
                drive.bl.setPower(-1);
                drive.br.setPower(1);
            }
            if (gamepad1.dpad_right) {
                drive.fl.setPower(-1);
                drive.fr.setPower(1);
                drive.bl.setPower(1);
                drive.br.setPower(-1);
            }



            telemetry.addData("Halfing Speed : ", pastX);
            telemetry.addData("Encoded Acceleration : ", drive.getEncodedAccel());

            telemetry.addData("Get Holon : ",
                    " FL: " + drive.getHolon(drive.fl) +
                            "FR :" + drive.getHolon(drive.fr) +
                            "BL : " + drive.getHolon(drive.bl) +
                            "BR : " + drive.getHolon(drive.br));
            telemetry.update();
*/

    }
}
