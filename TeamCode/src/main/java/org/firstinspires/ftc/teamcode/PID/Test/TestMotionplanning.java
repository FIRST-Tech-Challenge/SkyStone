package org.firstinspires.ftc.teamcode.PID.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Experimental.Movement.OdometryDrive;
import org.firstinspires.ftc.teamcode.Experimental.Movement.Trajectory;
import org.firstinspires.ftc.teamcode.Experimental.Units.Action;
import org.firstinspires.ftc.teamcode.Experimental.Units.Vector;
import org.firstinspires.ftc.teamcode.Experimental.Units.Vector.MoveBehavior;

import java.util.ArrayList;

@Config
@Autonomous(name = "MotionPlanningTest", group = "drive")
public class TestMotionplanning extends LinearOpMode {
    private OdometryDrive odoDrive;
    private Trajectory traj;
    private HardwareMap hwMap;

    public void runOpMode(){
        hwMap = new HardwareMap(hardwareMap);
        odoDrive = new OdometryDrive(hardwareMap, this);
        Vector initPos = new Vector(0, 0, odoDrive.getIMUHeading());
        ArrayList<Vector> vectors = new ArrayList<Vector>() {{
            add(new Vector(0.75, 0, 48, MoveBehavior.LineTo));
            //add(new Vector(0.75, 0, 12, Math.PI / 2, MoveBehavior.RotateTo));
            //add(new Vector(1, 12, 12, MoveBehavior.StrafeTo));
            //add(new Vector(intakeAction(1)));
            //add(new Vector(sleepAction(3000)));
            //add(new Vector(intakeAction(0)));
        }};
        traj = new Trajectory(initPos, vectors);

        while (!isStarted()){
            telemetry.addData("External Heading", odoDrive.getIMUHeading());
            telemetry.update();
        }

        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.resetEncoders();
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        runLoop();

        odoDrive.runTrajectory(traj);
    }

    private void runLoop(){
        Thread thread = new Thread(){
            public void run() {
                while (true) {
                    if(isStopRequested()) {
                        odoDrive.saveLogNow();
                        break;
                    }
                }
            }
        };
        thread.start();
    }

    private Action intakeAction(double power) {
        return new Action() {
            @Override
            public void run() {
                Thread thread = new Thread() {
                    public void run() {
                        hwMap.leftIntake.setPower(-power);
                        hwMap.rightIntake.setPower(power);
                    }
                };
                thread.start();
            }
        };
    }

    private Action sleepAction(long milisecs) {
        return new Action() {
            @Override
            public void run() {
                try{
                    Thread.sleep(milisecs);
                } catch (Exception e){}
            }
        };
    }
}
