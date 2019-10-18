package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.IMUSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.components.Vuforia.CameraChoice;


import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

import java.util.EnumMap;

public abstract class BaseStateMachine extends BaseOpMode {
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
                LOGGING
        }

        public enum Team {
                RED, BLUE
        }

        protected State mCurrentState;    // Current State Machine State.
        protected ColorSensor colorSensor;
        protected ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
        protected DistanceSensor distanceLeft, distanceRight, distanceFront;

        public void init(Team team) {
                super.init();
                super.setCamera(team == Team.RED ? CameraChoice.WEBCAM1 : CameraChoice.WEBCAM2);
                rearPerimeter = vuforia.targetsSkyStone.get(team == Team.RED ? 12 : 11);

//                distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
//                distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceRight");
//                distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceFront");

                newState(State.STATE_INITIAL);
        }

        @Override
        public void loop() {
                switch (mCurrentState) {
                        case LOGGING:
                                telemetry.addData("DistanceLeft", distanceLeft.getDistance(DistanceUnit.INCH));
                                telemetry.addData("Color Blue", colorSensor.blue());
                                telemetry.addData("Color Red", colorSensor.red());
                                telemetry.addData("Color Green", colorSensor.green());
                                telemetry.addData("Color Alpha", colorSensor.alpha());
                                telemetry.addData("Color Hue", colorSensor.argb());
                                telemetry.update();
                                break;
                        case STATE_INITIAL:
                                // Initialize
                                // Drive 0.5m (1 tile) to the left
                                driveSystem.driveToPosition(1000, DriveSystem.Direction.LEFT, 0.8);
                                driveSystem.turnAbsolute(0, 0.8);
                                newState(State.STATE_FIND_SKYSTONE);
                                mStateTime.reset();
                                break;

                        case STATE_FIND_SKYSTONE:
                                // Strafe towards line
                                // Identify SkyStone
                                telemetry.addData("State", "STATE_FIND_SKYSTONE");
                                if (mStateTime.seconds() > 8) {
                                        // TODO: Unable to detect stone after 10 seconds. Use dead reckoning
                                        // TODO: Make new state for this. Currently just set to log
                                        newState(State.LOGGING);
                                        break;
                                }

                                if (vuforia.isTargetVisible(skystone)) {
                                        newState(State.STATE_GRAB_STONE);
                                        break;
                                }
                                telemetry.update();
                                break;

                        case STATE_GRAB_STONE:
                                // Grab the stone and slurp it into the machine
                                telemetry.addData("State", "STATE_GRAB_STONE");

                                Orientation rotation = vuforia.getRobotHeading();
                                driveSystem.turn(rotation.thirdAngle - 90, 0.5);
                                if (vuforia.isTargetVisible(skystone)) {
                                        VectorF translation = vuforia.getRobotPosition();
                                        // Align with skystone
                                        driveSystem.driveToPosition((int) translation.get(1), DriveSystem.Direction.FORWARD, 0.25);
                                        // Offset to move other stones out of the way. One stone is 6.56 inches. I'm assuming the robot is 12 inches halfway
                                        // inch value: 6.56 + 12.0
                                        driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 0.8);
                                        // Move to the left and actually knock the stones out of the way
                                        // inch value: 56
                                        driveSystem.driveToPosition(1000, DriveSystem.Direction.LEFT, 0.8);
                                        driveSystem.turnAbsolute(0, 0.8);
                                        // TODO: Suck the stone in

                                        // Back up into the stone to suck it in
                                        driveSystem.driveToPosition(1000, DriveSystem.Direction.BACKWARD, 0.8);

                                        // TODO: Stone is sucked in and disable sucking

                                        // Prepare to deliver stone
                                        driveSystem.driveToPosition(1000, DriveSystem.Direction.RIGHT, 0.8);
                                        // Face same direction as the audience
                                        driveSystem.turnAbsolute(0, 0.8);
                                }
                                telemetry.update();
                                newState(State.STATE_DELIVER_STONE);
                                break;

                        case STATE_DELIVER_STONE:
                                telemetry.addData("State", "STATE_DELIVER_STONE");
                                // TODO: Use distance sensor to detect distance from wall
                                if (distanceLeft.getDistance(DistanceUnit.INCH) > 20) {
                                     //   driveSystem.drive((float) (driveSystem.imuSystem.getHeading() / 50.0), 0.0f, 0.0f, -0.5f);
                                } else {
                                        driveSystem.setMotorPower(0.0);
//                        newState(State.STATE_DEPOSIT_STONE);
                                }
                                telemetry.update();
                                break;

                        case STATE_FIND_STONE:
                                // Find a stone using TensorFlow
                                newState(State.STATE_GRAB_STONE);
                                break;

                        case STATE_PARK_AT_LINE:
                                // Find the line
                                // Park
                                break;

                        case STATE_DEPOSIT_STONE:
                                // Put stone down on foundation
                                newState(State.STATE_RETURN);
                                break;

                        case STATE_DRAG_FOUNDATION:
                                // Drag foundation out of box
                                newState(State.STATE_PARK_AT_LINE);
                                break;

                        case STATE_RETURN:
                                // Go back to block repository
                                newState(State.STATE_FIND_SKYSTONE);
                                break;
                }
        }


        public void newState(State newState) {
                // Restarts the state clock as well as the state
                mStateTime.reset();
                mCurrentState = newState;
        }

}

