package org.firstinspires.ftc.teamcode.opmodes;


import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.DriveSystem;

import java.util.EnumMap;

@Autonomous(name = "Test", group="Autonomous")
public class Test extends OpMode {
    public enum State {
        STATE_INITIAL,
        STATE_FIND_SKYSTONE,
        STATE_DELIVER_STONE,
        STATE_GRAB_STONE,
        STATE_FIND_STONE,
        STATE_PARK_AT_LINE,
        STATE_DEPOSIT_STONE,
        STATE_DRAG_FOUNDATION,
        STATE_RETURN,
    }

    private final String TAG = "DEBUG";

    public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.
    private DriveSystem driveSystem;
    private static final float mmPerInch = 25.4f;

    protected State mCurrentState;    // Current State Machine State.

    @Override
    public void init() {
        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }
        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));
        this.msStuckDetectLoop = 15000;
        newState(State.STATE_INITIAL);
    }


    // @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        switch (mCurrentState) {
            case STATE_INITIAL:
                Log.d(TAG, "STATE_INITIAL");
                telemetry.addData("State", "STATE_INITIAL");
                telemetry.update();
                newState(State.STATE_GRAB_STONE);
                break;

            case STATE_FIND_SKYSTONE:
                Log.d(TAG, "STATE_FIND_SKYSTONE");
                telemetry.addData("State", "STATE_FIND_SKYSTONE");
                telemetry.update();

                driveSystem.driveToPositionTicks(1650, DriveSystem.Direction.FORWARD, 0.5);
                driveSystem.counter++;
                Log.d(TAG, "driving forward");
                Log.d(TAG, "counter: " + driveSystem.counter);
                sleep(10000);

                driveSystem.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                driveSystem.driveToPositionTicks(1650, DriveSystem.Direction.BACKWARD, 0.5);
                driveSystem.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Log.d(TAG, "calling newState(GrabStone");
                newState(State.STATE_DELIVER_STONE);
                break;

            case STATE_GRAB_STONE:

                telemetry.addData("State", "STATE_GRAB_STONE");
                telemetry.update();

                Log.d(TAG, "STATE_GRAB_STONE");
                driveSystem.driveToPositionTicks(1000, DriveSystem.Direction.FORWARD, 0.5);
                //Log.d(TAG, ticks);
                sleep(10000);
                driveSystem.driveToPositionTicks(1000, DriveSystem.Direction.FORWARD, 0.5);
                sleep(10000);
                newState(State.STATE_DELIVER_STONE);
                break;

            case STATE_DELIVER_STONE:
                telemetry.addData("State", "STATE_DELIVER_STONE");
                telemetry.update();
                break;

            case STATE_FIND_STONE:
                telemetry.addData("State", "STATE_FIND_STONE");
                telemetry.update();
                newState(State.STATE_GRAB_STONE);
                break;

            case STATE_PARK_AT_LINE:
                telemetry.addData("State", "STATE_PARK_AT_LINE");
                telemetry.update();
                break;

            case STATE_DEPOSIT_STONE:
                telemetry.addData("State", "STATE_DEPOSIT_STONE");
                telemetry.update();
                newState(State.STATE_RETURN);
                break;

            case STATE_DRAG_FOUNDATION:
                telemetry.addData("State", "STATE_DRAG_FOUNDATION");
                telemetry.update();
                newState(State.STATE_PARK_AT_LINE);
                break;

            case STATE_RETURN:
                telemetry.addData("State", "STATE_RETURN");
                telemetry.update();
                newState(State.STATE_FIND_SKYSTONE);
                break;
        }
    }

    @Override
    public void stop() {
    }

    public void newState(State newState) {
        mCurrentState = newState;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}

