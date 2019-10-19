package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    double frHolo = 0;
    double flHolo = 0;

    double gyre;
    double bOffset;
    double cOffset;
    double primeSin;
    double alpha;

    boolean pastB = false;
    int quadrant;

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

        //Sets Hardware Map
        drive.fl = hardwareMap.dcMotor.get("fl");
        drive.fr = hardwareMap.dcMotor.get("fr");
        drive.bl = hardwareMap.dcMotor.get("bl");
        drive.br = hardwareMap.dcMotor.get("br");

        //Sets Motor Directions
        drive.fl.setDirection(DcMotor.Direction.FORWARD);
        drive.fr.setDirection(DcMotor.Direction.REVERSE);
        drive.bl.setDirection(DcMotor.Direction.FORWARD);
        drive.br.setDirection(DcMotor.Direction.REVERSE);

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

        double[] zeroRift;
        zeroRift = new double[2];

        speed = gamepad1.right_stick_x;

        telemetry.addData("Status", "Run Time: " + drive.runtime.toString());
        telemetry.addData("Motor Position", "Motor Rotation", +speed);

        if (Math.abs(gamepad1.left_stick_y) > .05) {
            leftStickY = gamepad1.left_stick_y;
        } else {
            leftStickY = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) > .05) {
            leftStickX = gamepad1.left_stick_x;
        } else {
            leftStickX = 0;
        }

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

        //Might be returning arrays wrong
        drive.hermite(zeroRift);

        velocity = zeroRift[1];
                //Math.hypot(leftStickX, leftStickY);
        direction = zeroRift[0];
                //Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        speed = gamepad1.right_stick_x;

        if (Math.abs(gamepad1.right_stick_x) < 0.075 ) {
            speed = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) > 0.075 ||
                Math.abs(gamepad1.left_stick_y) >
                0.075 || Math.abs(gamepad1.right_stick_x)
                > 0.075) {
            drive.fl.setPower((velocity - direction) - speed);
            drive.fr.setPower((velocity + direction) + speed);
            drive.bl.setPower((velocity + direction) - speed);
            drive.br.setPower((velocity - direction) + speed);
        }

        else {
            drive.snowWhite();
        }

                //drive.fl.setPower((velocity * Math.cos(direction) - speed) * speedProp);
                //drive.fr.setPower((velocity * Math.sin(direction) + speed) * speedProp);
                //drive.bl.setPower((velocity * Math.sin(direction) - speed) * speedProp);
                //drive.br.setPower((velocity * Math.cos(direction) + speed) * speedProp);



            telemetry.addData("Halfing Speed : ", pastX);
            telemetry.addData("Encoded Acceleration : ", drive.getEncodedAccel());

            telemetry.addData("Get Holon : ",
                            "FR :" + drive.getHolon(drive.fr) +
                            "BL : " + drive.getHolon(drive.bl) +
                            "BR : " + drive.getHolon(drive.br));
            telemetry.update();


        }
    }
