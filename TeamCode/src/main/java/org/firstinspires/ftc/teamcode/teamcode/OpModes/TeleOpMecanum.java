package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.teamcode.Hardware.Outtake;

@TeleOp(name="Arcade", group= "Tele Op")
public class TeleOpMecanum extends OpMode {

    DriveTrain drive = new DriveTrain();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    ElapsedTime time = new ElapsedTime();

    double direction;
    double velocity;
    double speed;
    double speedProp = 1.0;
    boolean pastX = false;

    // CFM variables

    private static final double  massFoundation = 1.905; // Mass in kg
    private static final double massStone = .1882;
    static final double muBlocks = .78;
    static final double muMat = .535;
    double fix = 1.0;
    double tolerance = .05;
    double mass = 0.0;
    double foundationFriction = 0.0;
    double maxCFM_Velocity = 0.0;
    double CFM_AngularVelocity = 0.0;
    double cfm_power = 0.0;

    int numberStackedBlocks = 0;


    //  Game pad Control Stick Variables
    double right_stick_x;
    double left_stick_x;
    double left_stick_y;

    @Override
    public void init() {

        drive.fl = hardwareMap.dcMotor.get("fl");
        drive.fr = hardwareMap.dcMotor.get("fr");
        drive.bl = hardwareMap.dcMotor.get("bl");
        drive.br = hardwareMap.dcMotor.get("br");

        drive.fl.setDirection(DcMotor.Direction.FORWARD);
        drive.fr.setDirection(DcMotor.Direction.REVERSE);
        drive.bl.setDirection(DcMotor.Direction.FORWARD);
        drive.br.setDirection(DcMotor.Direction.REVERSE);

        drive.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.initIntakeTele(this);
        outtake.initOuttake(this);


        drive.runtime.reset();
        time.reset();
    }

    //Main Loop
    @Override
    public void loop() {

        //  Set Join Sticks for Arcade Drive


            left_stick_y = gamepad1.left_stick_y;
            left_stick_x = gamepad1.left_stick_x;
            right_stick_x = gamepad1.right_stick_x;




        /*if (gamepad1.x != pastX) {
            pastX = gamepad1.x;
            if (gamepad1.x) {
                if (speedProp == 1) {
                    speedProp = 0.5;
                } else {
                    speedProp = 1;
                }
            }
        }*/



            /*//Foundation Moving Toggle
            //Toggle sets speed such that the robot can move the fastest
            //while moving the foundation and not dropping any blocks
            //Takes into account the mass of the foundation and block stack
            //and the friction of the floor


            numberStackedBlocks = outtake.getNumberOfBlocks();
            //  Mass of Whole Object
            mass = massFoundation + numberStackedBlocks * massStone;

            //  Max CFM velocity, calculated
            maxCFM_Velocity = fix * Math.sqrt((2 * tolerance * 9.81 * massStone * numberStackedBlocks * muBlocks)
                    / mass);

            //  CFM velocity to Angular Velocity
            CFM_AngularVelocity = maxCFM_Velocity / (DriveTrain.wheelDiam / 2);

            //  Power to set motors to follow CFM velocity.
            cfm_power = (-1) * (DriveTrain.stallTorque / DriveTrain.noLoadSpeed) * CFM_AngularVelocity
                    + DriveTrain.stallTorque * CFM_AngularVelocity;

            telemetry.addData("Number of Blocks : ", numberStackedBlocks);*/



        if(gamepad1.x)
        {
            time.reset();
            while(time.milliseconds() < 300){ }
            if(speedProp == 1)
            {
                speedProp = .5;
            }
            else {
                speedProp = 1;
            }
        }

        telemetry.addData("Speed", speedProp);
        telemetry.addData("Vertical", drive.getRadiaxVertical());
        telemetry.addData("Hypotenuse", drive.getRadiaxHypotenuse());
        telemetry.addData("Horizontal", drive.getRadiaxHorizontal());
        telemetry.addData("Radiax", drive.getRadiax());
        //telemetry.addData("Vector", drive.getVector());


        if(gamepad1.dpad_left)
        {
            drive.setStrafePower(-1);
        }
        else if(gamepad1.dpad_right)
        {
            drive.setStrafePower(1);
        }
        else if (Math.abs(left_stick_x) > 0.05 ||
                 Math.abs(left_stick_y) > 0.05 ||
                 Math.abs(right_stick_x) > 0.05) {

            drive.fl.setPower(speedProp * ((left_stick_y - left_stick_x) - right_stick_x));
            drive.fr.setPower(speedProp * ((left_stick_y + left_stick_x) + right_stick_x));
            drive.bl.setPower(speedProp * (left_stick_y + left_stick_x) - right_stick_x);
            drive.br.setPower(speedProp * (left_stick_y - left_stick_x) + right_stick_x);
        }
        else {
            drive.snowWhite();
        }

        intake.Intake_TeleOp();

        outtake.outTake_TeleOp(this);

        telemetry.update();
    }
}
