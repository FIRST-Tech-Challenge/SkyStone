package org.firstinspires.ftc.teamcode.opmodes;

    /* Copyright (c) 2014 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;

import java.util.EnumMap;

@TeleOp(name = "Vuforia", group="Autonomous")
public class BaseStateMachine extends OpMode {
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

        public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.

        private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
        private Vuforia vuforia;
        private VuforiaTrackable skystone;
        private DriveSystem driveSystem;
        private static final float mmPerInch = 25.4f;

        protected State mCurrentState;    // Current State Machine State.

        @Override
        public void init() {
            vuforia = new Vuforia(hardwareMap, Vuforia.CameraChoice.PHONE_BACK);
            skystone = vuforia.targetsSkyStone.get(0);
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
                    // Initialize
                    newState(State.STATE_FIND_SKYSTONE);
                    break;

                case STATE_FIND_SKYSTONE:
                    // Strafe towards line
                    // Identify SkyStone
                    // If we can't see it after 3 feet, start driving  until we do
                    telemetry.addData("State", "STATE_FIND_SKYSTONE");

                    if (!vuforia.isTargetVisible(skystone)) {
                        driveSystem.setMotorPower(0.2);
                    } else {
                        driveSystem.setMotorPower(0);
                        newState(State.STATE_GRAB_STONE);
                    }
                    telemetry.update();

                    break;

                case STATE_GRAB_STONE:
                    // Grab the stone and slurp it into the machine
                    telemetry.addData("State", "STATE_GRAB_STONE");

                    Orientation rotation = vuforia.getRobotHeading();
                    driveSystem.turn(rotation.thirdAngle - 90, 0.8);
                    if (vuforia.isTargetVisible(skystone)) {
                        VectorF translation = vuforia.getRobotPosition();
                        driveSystem.driveToPositionInches(translation.get(1) / mmPerInch, DriveSystem.Direction.FORWARD, 0.5);
                    }
                    driveSystem.turn(-90, 1.0);
                    driveSystem.driveToPositionInches(24, DriveSystem.Direction.FORWARD, 1.0);
                    telemetry.update();

                    newState(State.STATE_DELIVER_STONE);
                    break;

                case STATE_DELIVER_STONE:
                    telemetry.addData("State", "STATE_DELIVER_STONE");

                    // Drive with stone to the foundation
                    // Go under bridge
                    telemetry.update();
//                    newState(State.STATE_DEPOSIT_STONE);
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

        // Stop
        @Override
        public void stop() {
        }

    public void newState(State newState) {
        // Restarts the state clock as well as the state
        mStateTime.reset();
        mCurrentState = newState;
    }

}

