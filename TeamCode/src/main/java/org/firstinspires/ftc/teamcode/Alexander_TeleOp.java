    /* Copyright (c) 2017 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */

    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import java.util.stream.Collector;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    //import com.qualcomm.robotcore.util.*;
    //import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
    //import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    //import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    //import com.qualcomm.robotcore.hardware.ColorSensor;
    //import com.qualcomm.robotcore.hardware.DcMotor;
    //import com.qualcomm.robotcore.hardware.I2cAddr;
    //import com.qualcomm.robotcore.hardware.I2cDevice;
    //import com.qualcomm.robotcore.hardware.Servo;
    //import com.qualcomm.robotcore.util.ElapsedTime;

    /**
     * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
     * All device access is managed through the HardwarePushbot class.
     * The code is structured as a LinearOpMode
     *
     * This particular OpMode executes a POV Game style Teleop for a PushBot
     * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
     * It raises and lowers the claw using the Gampad Y and A buttons respectively.
     * It also opens and closes the claws slowly using the left and right Bumper buttons.
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @TeleOp(name="Alexander_TeleOp", group="Linear OpMode")
    //@Disabled
    public class Alexander_TeleOp extends LinearOpMode {


        private DcMotor getNewMotor(String motorName) { //these could be made generic using type notation
            try {
                return (hardwareMap.get(DcMotor.class, motorName));
            } catch (Exception e) {
                telemetry.addData("MOTOR: "+motorName, "   offline");
                telemetry.update();
                return (null);
            }
        }

        //Speed Factors for Fast/Slow Mode
        private double speedFactor = 1.0; //default full speed
        private ElapsedTime runtime = new ElapsedTime();
        private double currentLoopTime = 0.0;
        private double lastLoopTime = 0.0;
        private double direction = 1.0; //default forward


        //Driving Motors
        private DcMotor frontLeft = null;
        private DcMotor frontRight = null;
        private DcMotor backLeft = null;
        private DcMotor backRight = null;

        //Attachment Motors
        private DcMotor collectorLeft = null;
        private DcMotor collectorRight = null;
        private DcMotor linearSlide = null;
        private DcMotor guidance = null;
        //Attachment Servos
        private Servo clamp = null;
        private Servo rotation = null;
        private Servo foundation = null;
        private Servo release = null;


        //other variables
        private static boolean tele = true; //show telemetry
        double lastPosition = 0; // used for Block rotation




        //Our software coach from last year helped us with this method that uses trigonometry to operate mecanum wheels
        private void mecanumMove(double leftStickX, double leftStickY, double rightStickX) {


            double distanceFromCenter = Math.sqrt(leftStickY * leftStickY + leftStickX * leftStickX);  // might be leftStickY * leftStickX This double uses the pythagorean theorem to find  out the distance from the the joystick center

            double robotAngle = Math.atan2(-1 * leftStickY, leftStickX) - Math.PI / 4;

            final double frontLeftPower = distanceFromCenter * Math.cos(robotAngle) + rightStickX;    //Multiplies the scaling of the joystick to give different speeds based on joystick movement
            final double frontRightPower = distanceFromCenter * Math.sin(robotAngle) - rightStickX;
            final double backLeftPower = distanceFromCenter * Math.sin(robotAngle) + rightStickX;
            final double backRightPower = distanceFromCenter * Math.cos(robotAngle) - rightStickX;

            if(frontLeft != null)
                frontLeft.setPower(frontLeftPower);
            if(frontRight != null)
                frontRight.setPower(frontRightPower);
            if(backLeft != null)
                backLeft.setPower(backLeftPower);
            if(backRight != null)
                backRight.setPower(backRightPower);
        }
        private void Collector(boolean collectorIn, boolean collectorOut, boolean guidanceIn, boolean guidanceOut )
        {
            if(collectorIn)
            {

                if(collectorLeft != null) {
                    collectorLeft.setPower(1.0);
                }
                if(collectorRight != null) {
                    collectorRight.setPower(1.0);
                }

            }
            else if(collectorOut)
            {
                if(collectorLeft != null) {
                    collectorLeft.setPower(-1.0);
                }
                if(collectorRight != null) {
                    collectorRight.setPower(-1.0);
                }

            }

            if(guidanceIn)
            {

                if(guidance != null) {
                    guidance.setPower(1.0);
                }

            }
            else if(guidanceOut)
            {

                if(guidance != null) {
                    guidance.setPower(-1.0);
                }

            }
            else
            {
                if(collectorLeft != null) {
                    collectorLeft.setPower(0.0);
                }
                if(collectorRight != null) {
                    collectorRight.setPower(0.0);
                }
                if(guidance != null) {
                    guidance.setPower(0.0);
                }
            }

        }

        private void SkystonePositioner(double rightStickY)
        {
            if(linearSlide != null) {
                linearSlide.setPower(rightStickY);
            }
        }

        private void moveClamps(boolean dPadUp, boolean dPadDown)
        {
            if (dPadUp) {

                //clamp.setDirection(DcMotorSimple.Direction.FORWARD);
                clamp.setPosition(0.53);
                telemetry.addData("Status: ","Up Activated" );

            }
            else if (dPadDown)
            {

                //     clamp.setDirection(DcMotorSimple.Direction.REVERSE);
                clamp.setPosition(0.63);
                telemetry.addData("Status: ","Down Activated" );

            }
            //*   else
            //    {
            //        clamp.setPower(0.0);
            //        telemetry.addData("Status: "," Neutral Activated" );

            //    }
            telemetry.update();
        }
        private void MoveHook(boolean Up, boolean Down) {
            if (Up) {

                foundation.setPosition(1.0);

            }
            else if (Down) {

                foundation.setPosition(0.4);

            }
        }
        private void ReleaseCollector(boolean released) {

            if (released) {

                release.setPosition(-1.0);

            }
            else
            {

                release.setPosition(1.0);

            }

        }
        private void RotateBlock(boolean rotateLeft, boolean rotateRight) {

            if (rotateLeft) {

                rotation.setPosition(0.875);

            }
            else if (rotateRight) {

                rotation.setPosition(-0.7);


            }

        }
        @Override
        public void runOpMode() throws InterruptedException{


            telemetry.addData("Droid", "Robot");



            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */
            //robot.init(hardwareMap);

            //Above line is commented out because Hardware map is used for accessories such as attachment sensors/servos/motors

            // Send telemetry message to signify robot waiting;
            //telemetry.addData("Say", "Hello Driver");




            //initialize required driving motors
            frontLeft = getNewMotor("lf");
            frontRight = getNewMotor("rf");
            backLeft = getNewMotor("lb");
            backRight = getNewMotor("rb");



            //init servos
            clamp = hardwareMap.servo.get("clamp");
            foundation = hardwareMap.servo.get("foundation");
            rotation = hardwareMap.servo.get("rotation");
            release = hardwareMap.servo.get("release");



            //Init Accessory Motors
            collectorLeft = getNewMotor("lla");
            collectorRight = getNewMotor("rla");
            collectorRight = getNewMotor("rla");
            linearSlide = getNewMotor("elevator");
            guidance = getNewMotor("guidance");


            if(frontLeft != null)
                frontLeft.setDirection(DcMotor.Direction.FORWARD);           // This makes the front of the robot the side with the block intake!!!!!
            if(frontRight != null)                                          // This makes the front of the robot the side with the block intake!!!!!
                frontRight.setDirection(DcMotor.Direction.REVERSE);
            if(backLeft != null)
                backLeft.setDirection(DcMotor.Direction.FORWARD);
            if(backRight != null)
                backRight.setDirection(DcMotor.Direction.REVERSE);

            if(collectorLeft != null)
                collectorLeft.setDirection(DcMotor.Direction.REVERSE);
            if(collectorRight != null)
                collectorRight.setDirection(DcMotor.Direction.FORWARD);

            if(linearSlide != null)
                linearSlide.setDirection(DcMotor.Direction.FORWARD);
            if(guidance!= null)
                guidance.setDirection(DcMotor.Direction.FORWARD);


            rotation.setPosition(0.9);


            telemetry.addData("Status", "Initialized");                 //Telemetry is the messages displayed on phone
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();    //resets runtime()

            currentLoopTime = runtime.time();
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                lastLoopTime = currentLoopTime;
                currentLoopTime = runtime.time();

                //gamepad1
                double forwardBack = gamepad1.left_stick_y;
                double leftRight = gamepad1.left_stick_x;
                double Rotate = gamepad1.right_stick_x;
                boolean slowMode = (gamepad1.left_trigger > 0.2);
                boolean reverseDirection = gamepad1.a;
                boolean hookUp = gamepad1.dpad_up;
                boolean hookDown = gamepad1.dpad_down;
                boolean released = gamepad1.x;




                //gamepad2
                boolean collectorIn = gamepad2.right_trigger > 0.2;
                boolean collectorOut = gamepad2.right_bumper;
                boolean guidanceIn = gamepad2.left_trigger > 0.2;
                boolean guidanceOut = gamepad2.left_bumper;

                double StoneUpDown = gamepad2.right_stick_y;
                boolean clampsIn = gamepad2.dpad_up;
                boolean clampsOut = gamepad2.dpad_down;
                boolean rotateLeft = gamepad2.x;
                boolean rotateRight = gamepad2.b;






                if (slowMode) {
                    speedFactor = 0.25;
                } else {
                    speedFactor = 1.0;
                }
                if (reverseDirection) {
                    direction = -1;
                } else {
                    direction = 1;
                }

                double leftStickY = forwardBack * speedFactor * direction;
                double leftStickX = leftRight * speedFactor * direction;
                double rightStickX = Rotate * speedFactor * direction;



                //Methods
                mecanumMove(leftStickX, leftStickY, rightStickX);
                Collector(collectorIn, collectorOut, guidanceIn, guidanceOut);
                SkystonePositioner(StoneUpDown);
                moveClamps(clampsIn, clampsOut);
                RotateBlock(rotateLeft, rotateRight);
                MoveHook(hookUp, hookDown);
                ReleaseCollector(released);



                //Telemetry
                if (tele) telemetry.addData("Status", "Run Time: " + runtime.toString());
                if (tele) telemetry.update();

            }
        }
    }