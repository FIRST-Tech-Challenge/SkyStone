package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JonasMotor", group = "TestArea")


public class RoboTest1 extends OpMode {

    DcMotor first = null;
    DcMotor second = null;
    DcMotor third = null;
    DcMotor fourth = null;

    @Override
    public void init() {

        /**
         * Initialize the hardware variables.
         */
        first = hardwareMap.dcMotor.get("first");
        second = hardwareMap.dcMotor.get("second");
        third = hardwareMap.dcMotor.get("third");
        fourth = hardwareMap.dcMotor.get("fourth");

        // same?
        /*
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
         */

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftDrive.setDirection(DcMotor.Direction.FORWARD);
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        /**
         * smoothThePower - terminal test
         */
        for (int i = 0; i < 100; i++)
            System.out.println(i / 100 + " -> " + smoothThePower(i / 100));

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry
        double firstPower;
        double secondPower;
        double thirdPower;
        double fourthPower;

        first.setPower(gamepad1.left_stick_x);
        second.setPower(-gamepad1.left_stick_x);
        third.setPower(gamepad1.left_stick_y);
        fourth.setPower(-gamepad1.left_stick_y);

        first.setPower(gamepad1.right_stick_x);
        second.setPower(gamepad1.right_stick_x);
        third.setPower(gamepad1.left_stick_y);
        fourth.setPower(gamepad1.left_stick_y);

        // Get the current motor target position
        // first.getTargetPosition();

        // Show the elapsed game time and wheel power.
        // telemetry.addData("Status", "Run Time: " + runtime.toString());
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

    }

    /**
     * a tangens hyperbolic function for a smooth gaming experience 
     * @param p
     * @return 0.390911 * tanh (6.66 * (p - 0.5))) + 0.5 )
     */
    private double smoothThePower(double p){
        return (0.390911*(java.lang.Math.tanh(6.66*(p-0.5)))+0.5);
    }

}
