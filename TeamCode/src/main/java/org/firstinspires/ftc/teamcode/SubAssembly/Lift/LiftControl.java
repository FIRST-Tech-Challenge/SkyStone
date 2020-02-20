package org.firstinspires.ftc.teamcode.SubAssembly.Lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class LiftControl {/* Constants */
    final double LIFT_SPEED = 1.0;

    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */

    Thread updateThread = new updateThread();
    private enum Modes {STOP, UP, DOWN}
    private Modes mode = Modes.STOP;

    private DcMotor LifterRightM;
    private DcMotor LifterLeftM;
    private TouchSensor LifterButtonT;
    private TouchSensor LifterButtonB;
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare public class object */


    /* Subassembly constructor */
    public LiftControl() {
    }

    public void initialize(LinearOpMode opMode) {
        HardwareMap hwMap;

        opMode.telemetry.addLine("Lift Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        /* Map hardware devices */
        LifterRightM = hwMap.dcMotor.get("LifterRightM");
        LifterLeftM = hwMap.dcMotor.get("LifterLeftM");
        LifterRightM.setDirection(DcMotor.Direction.FORWARD);
        LifterLeftM.setDirection(DcMotor.Direction.FORWARD);
        LifterRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LifterLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LifterRightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LifterLeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LifterRightM.setPower(0);
        LifterLeftM.setPower(0);

        LifterButtonB = hwMap.touchSensor.get("LifterButtonB");
        LifterButtonT = hwMap.touchSensor.get("LifterButtonT");

        updateThread.start();
    }

    /* Subassembly destructor */
    public void finalize() {
        updateThread.interrupt();
    }

    public void MoveUp() {
        // set thread mode to up
        mode = Modes.UP;
//        LifterLeftM.setPower(LIFT_SPEED);
//        LifterRightM.setPower(LIFT_SPEED);
    }

    public void MoveDown() {
        // set thread mode to down
        mode = Modes.DOWN;
//        LifterLeftM.setPower(-LIFT_SPEED);
//        LifterRightM.setPower(-LIFT_SPEED);
    }

    public void Stop() {
        // set thread mode to stop and stop motors immediately
        mode = Modes.STOP;
        LifterLeftM.setPower(0);
        LifterRightM.setPower(0);
    }

    public void TimeDelay(double delayTimeSEC) {
        double startTime = 0;
        double elapsedTime = 0;
        startTime = runtime.seconds();
        do {
            elapsedTime = runtime.seconds() - startTime;
            opmode.sleep(40);
        } while ((elapsedTime < delayTimeSEC) && !opmode.isStopRequested());
    }

    public void MoveUpTime(double time) {
        mode = Modes.UP;
//        LifterLeftM.setPower(LIFT_SPEED);
//        LifterRightM.setPower(LIFT_SPEED);
        TimeDelay(time);
        Stop();
    }

    public void MoveDownTime(double time) {
        mode = Modes.DOWN;
//        LifterLeftM.setPower(-LIFT_SPEED);
//        LifterRightM.setPower(-LIFT_SPEED);
        TimeDelay(time);
        Stop();
    }

    private boolean isLimitTop() {
        // !!! current hardware configuration has isPressed returning
        //     TRUE when the limit switch is NOT pressed
        return !LifterButtonT.isPressed();
    }

    private boolean isLimitBottom() {
        // !!! current hardware configuration has isPressed returning
        //     TRUE when the limit switch is NOT pressed
        return !LifterButtonB.isPressed();
    }

    private class updateThread extends Thread {
        public updateThread() {
            mode = Modes.STOP;
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    switch (mode) {
                        default:
                        case STOP:
                            Stop();
                            break;
                        case UP:
                            if (isLimitTop()) {
                                Stop();
                            } else {
                                LifterLeftM.setPower(LIFT_SPEED);
                                LifterRightM.setPower(LIFT_SPEED);
                            }
                            break;
                        case DOWN:
                            if (isLimitBottom()) {
                                Stop();
                            } else {
                                LifterLeftM.setPower(-LIFT_SPEED);
                                LifterRightM.setPower(-LIFT_SPEED);
                            }
                            break;
                    }

                    Thread.sleep(40);
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            catch (InterruptedException e) {
            }
        }
    }
}
