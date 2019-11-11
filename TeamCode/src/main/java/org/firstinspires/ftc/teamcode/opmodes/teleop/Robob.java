package org.firstinspires.ftc.teamcode.opmodes.teleop;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.parts.ManyMotors;
import org.firstinspires.ftc.teamcode.parts.SeveralServos;
import org.firstinspires.ftc.teamcode.parts.WeirdWheels;
import org.firstinspires.ftc.teamcode.parts.AwesomeArm;
import org.firstinspires.ftc.teamcode.util.glob.SharedTelemetry;

@TeleOp(name="Robob", group="Game Controller")
public class Robob extends OpMode
{
    private WeirdWheels wheels;

    private ManyMotors intake;
    private boolean intakeBtn;
    private boolean intakeOpen;

    private AwesomeArm arm;
    private boolean grippersBtn;

    private SeveralServos bulldozer;
    private boolean bulldozerBtn;
    private boolean bulldozerDown;

    private Servo door;
    private boolean doorBtn;
    private boolean doorDown = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        SharedTelemetry.telemetry = telemetry;

        telemetry.addData("Status", "Initialized");

        // wheels
        DcMotor fl              = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr              = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl              = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br              = hardwareMap.get(DcMotor.class, "br");
        List<DcMotor> wheelList = new ArrayList<>(Arrays.asList(fl, fr, bl, br));
        this.wheels = new WeirdWheels(wheelList);

        // intake
        DcMotor i1               = hardwareMap.get(DcMotor.class, "i1");
        DcMotor i2               = hardwareMap.get(DcMotor.class, "i2");
        List<DcMotor> intakeList = new ArrayList<>(Arrays.asList(i1,i2));
        this.intake = new ManyMotors(intakeList);

        // arm
        DcMotor shoulder     = hardwareMap.get(DcMotor.class, "shoulder");
        DcMotor actuator     = hardwareMap.get(DcMotor.class, "actuator");
        Servo elbow          = hardwareMap.get(Servo.class, "elbow");
        Servo lGrip          = hardwareMap.get(Servo.class, "lGrip");
        Servo rGrip          = hardwareMap.get(Servo.class, "rGrip");
        lGrip.setDirection(Servo.Direction.REVERSE);
        rGrip.setDirection(Servo.Direction.FORWARD);
        List<Servo> grippers = new ArrayList<>(Arrays.asList(lGrip,rGrip));
        this.arm = new AwesomeArm(shoulder,actuator,elbow,grippers);

        lGrip.setPosition(0);
        rGrip.setPosition(0);

        DcMotor i = intake.motors.get(0);

        i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (i.getCurrentPosition() != 0) {}
        i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        i.setPower(0);
        i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo lbd                 = hardwareMap.get(Servo.class, "lbd");
        Servo rbd                 = hardwareMap.get(Servo.class, "rbd");
        lbd.setDirection(Servo.Direction.REVERSE);
        List<Servo> bulldozerList = new ArrayList<>(Arrays.asList(lbd,rbd));
        this.bulldozer = new SeveralServos(bulldozerList);

        this.door = hardwareMap.get(Servo.class, "door");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("grip1p",arm.grippers.get(0).getPosition());
        telemetry.addData("grip2p",arm.grippers.get(1).getPosition());
        telemetry.addData("act",arm.actuator.getCurrentPosition());

        wheelControl();
        intakeControl();
        armControl();
        bulldozerControl();
        doorControl();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        wheels.stop();
    }

    private void wheelControl() {
        double strafe   = gamepad1.left_stick_x;
        double vertical = gamepad1.left_stick_y;
        double rotate   = gamepad1.right_stick_x;

        wheels.setVelocityXYR(strafe,vertical,rotate);
        wheels.moveMecanum();

    }

    private void intakeControl() {
        boolean a = gamepad2.a;
        if(a && a != intakeBtn) {
            intakeOpen = !intakeOpen;
        }
        intakeBtn = a;

        if(intakeOpen) {
            intake.move(new ArrayList<>(Collections.nCopies(2, 0.5)));
        } else {
            intake.move(new ArrayList<>(Collections.nCopies(2, 0.0)));
        }

    }

    private void armControl() {
        double shoulderPwr = gamepad2.left_stick_y;
        arm.moveShoulder(shoulderPwr/5);

        double actuatorUp = gamepad2.right_trigger-gamepad2.left_trigger;
        arm.moveActuator(actuatorUp);
        telemetry.addData("pos",arm.actuatorPos);

        double elbowPwr = gamepad2.right_stick_y;
        arm.moveElbow(elbowPwr);

        boolean b = gamepad2.b;
        if(b && b != grippersBtn) {
            arm.toggleGrippers();
        }
        grippersBtn = b;
        telemetry.addData("open",arm.grippersOpen);
    }

    private void bulldozerControl() {
        boolean y = gamepad1.y;
        if(y && y != bulldozerBtn) {
            bulldozerDown = !bulldozerDown;
        }
        bulldozerBtn = y;

        if(bulldozerDown) {
            bulldozer.setPosition(new ArrayList<>(Collections.nCopies(2, 0.2)));
        } else {
            bulldozer.setPosition(new ArrayList<>(Collections.nCopies(2, 1.0)));
        }

        if(gamepad1.dpad_up) {
            bulldozer.setPosition(new ArrayList<>(Collections.nCopies(2, 0.0)));
        }
    }

    private void doorControl() {
        boolean x = gamepad1.x;
        if(x && x != doorBtn) {
            doorDown = !doorDown;
        }
        doorBtn = x;

        if(doorDown) {
            door.setPosition(0.5);
        } else {
            door.setPosition(0.0);
        }
    }
}