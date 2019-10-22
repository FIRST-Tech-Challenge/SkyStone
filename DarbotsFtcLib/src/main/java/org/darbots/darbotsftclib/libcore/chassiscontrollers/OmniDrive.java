package org.darbots.darbotsftclib.libcore.chassiscontrollers;

import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionTracker;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.tasks.motor_tasks.RobotFixCountSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.tasks.motor_tasks.RobotFixedSpeedTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedTurnTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedXDistanceTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedZDistanceTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTeleOpControlTask;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotorTaskCallBack;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class OmniDrive extends RobotMotionSystem {
    public static class FixedXDistanceTask extends RobotMotionSystemFixedXDistanceTask{
        private OmniDrive m_Drive;
        private int m_CountsToMove = 0;
        private int m_LTStartCount = 0;
        private int m_RTStartCount = 0;
        private int m_LBStartCount = 0;
        private int m_RBStartCount = 0;
        private float m_StartDeg = 0;
        public FixedXDistanceTask(OmniDrive ODrive, double XDistance, double Speed) {
            super(XDistance, Speed);
            this.m_Drive = ODrive;
        }

        public FixedXDistanceTask(FixedXDistanceTask Task) {
            super(Task);
            this.m_Drive = Task.m_Drive;
        }

        @Override
        protected void __startTask() {
            __recalculateCounts();

            if(GlobalUtil.getGyro() != null){
                GlobalUtil.getGyro().updateStatus();
                m_StartDeg = m_Drive.isGyroGuidedDrivePublicStartingAngleEnabled() ? m_Drive.getGyroGuidedDrivePublicStartingAngle() : XYPlaneCalculations.normalizeDeg(GlobalUtil.getGyro().getHeading());
            }

            double AbsSpeed = Math.abs(getSpeed());
            if(m_Drive.isSteadySpeedUp() && AbsSpeed > this.getMotionSystem().getSteadySpeedUpThreshold()){
                AbsSpeed = 0;
            }
            double LTSpeed = this.m_CountsToMove > 0 ? -AbsSpeed : AbsSpeed;
            double RTSpeed = this.m_CountsToMove > 0 ? -AbsSpeed : AbsSpeed;
            double LBSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;
            double RBSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;

            m_LTStartCount = m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount();
            m_RTStartCount = m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount();
            m_LBStartCount = m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount();
            m_RBStartCount = m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount();


            m_Drive.m_LeftTopMotor.getMotorController().getMotor().setPower(LTSpeed);
            m_Drive.m_RightTopMotor.getMotorController().getMotor().setPower(RTSpeed);
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().setPower(LBSpeed);
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().setPower(RBSpeed);

            m_Drive.m_LeftTopMotor.getMotorController().getMotor().setCurrentMovingType(RobotMotor.MovingType.withSpeed);
            m_Drive.m_RightTopMotor.getMotorController().getMotor().setCurrentMovingType(RobotMotor.MovingType.withSpeed);
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().setCurrentMovingType(RobotMotor.MovingType.withSpeed);
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().setCurrentMovingType(RobotMotor.MovingType.withSpeed);
        }

        private void __recalculateCounts(){
            int CountToMove = (int) Math.round(this.getXDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getXPerCounterClockwiseDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.getLinearMotionDistanceFactor());
            CountToMove = Math.abs(CountToMove);
            int CountToMoveRight = (this.getXDistance() >= 0 ? CountToMove : -CountToMove);
            this.m_CountsToMove = CountToMoveRight;
        }

        @Override
        protected void __taskFinished() {
            if(this.m_Drive.getPositionTracker() != null) {
                int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;
                double DistanceMoved = CountsMoved / m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getRobotWheel().getXPerCounterClockwiseDistance() / m_Drive.getLinearMotionDistanceFactor();
                getMotionSystem().getPositionTracker().drive_MoveThroughRobotAngle(
                        -90,
                        DistanceMoved
                );
            }
        }

        @Override
        public void updateStatus() {
            m_Drive.m_LeftTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().updateStatus();
            //Left / Right Speed Control
            if(this.isBusy()){
                double AbsSpeed = Math.abs(getSpeed());


                if(this.getMotionSystem().isSteadySpeedUp() && AbsSpeed > this.getMotionSystem().getSteadySpeedUpThreshold()){
                    double ExtraSpeed = AbsSpeed - this.getMotionSystem().getSteadySpeedUpThreshold();

                    int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;

                    double jobPercentile = Math.abs( ((double) CountsMoved) / m_CountsToMove);

                    if(jobPercentile < 0.30){
                        AbsSpeed = this.getMotionSystem().getSteadySpeedUpThreshold() + (jobPercentile / 0.30) * ExtraSpeed;
                    }else if(jobPercentile > 0.70){
                        AbsSpeed = this.getMotionSystem().getSteadySpeedUpThreshold() + ((1.0 - jobPercentile) / 0.30) * ExtraSpeed;
                    }
                }

                double LTSpeed = this.m_CountsToMove > 0 ? -AbsSpeed : AbsSpeed;
                double RTSpeed = this.m_CountsToMove > 0 ? -AbsSpeed : AbsSpeed;
                double LBSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;
                double RBSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;

                if(GlobalUtil.getGyro() != null && m_Drive.isGyroGuidedDriveEnabled()) {
                    GlobalUtil.getGyro().updateStatus();
                    double currentAng = GlobalUtil.getGyro().getHeading();
                    double deltaAng = XYPlaneCalculations.normalizeDeg(currentAng - m_StartDeg);
                    if (GlobalUtil.getGyro().getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.Clockwise) {
                        deltaAng = -deltaAng;
                    }


                    double deltaSpeedEachSide = 0;
                    if (Math.abs(deltaAng) >= 5) {
                        deltaSpeedEachSide = Range.clip(0.5 * AbsSpeed, 0, 0.25);
                    } else if (Math.abs(deltaAng) >= 3) {
                        deltaSpeedEachSide = Range.clip(0.25 * AbsSpeed, 0, 0.2);
                    } else if (Math.abs(deltaAng) >= 1) {
                        deltaSpeedEachSide = Range.clip(0.1 * AbsSpeed, 0, 0.1);
                    } else if (Math.abs(deltaAng) >= 0.5) {
                        deltaSpeedEachSide = Range.clip(0.05 * AbsSpeed, 0, 0.05);
                    }
                    if (deltaSpeedEachSide < 0.025 && deltaSpeedEachSide != 0) {
                        deltaSpeedEachSide = 0.025;
                    }


                    if (deltaAng > 0) {
                        LTSpeed -= deltaSpeedEachSide;
                        RTSpeed -= deltaSpeedEachSide;
                        LBSpeed -= deltaSpeedEachSide;
                        RBSpeed -= deltaSpeedEachSide;
                    } else if (deltaAng < 0) {
                        LTSpeed += deltaSpeedEachSide;
                        RTSpeed += deltaSpeedEachSide;
                        LBSpeed += deltaSpeedEachSide;
                        RBSpeed += deltaSpeedEachSide;
                    }
                }
                m_Drive.m_LeftTopMotor.getMotorController().getMotor().setPower(LTSpeed);
                m_Drive.m_RightTopMotor.getMotorController().getMotor().setPower(RTSpeed);
                m_Drive.m_LeftBottomMotor.getMotorController().getMotor().setPower(LBSpeed);
                m_Drive.m_RightBottomMotor.getMotorController().getMotor().setPower(RBSpeed);
            }

            //Distance / Count Control
            if(this.isBusy()) {
                if (this.m_CountsToMove > 0) {
                    if (
                            m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() <= m_LTStartCount - m_CountsToMove ||
                                    m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount() <= m_RTStartCount - m_CountsToMove ||
                                    m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount() >= m_LBStartCount + m_CountsToMove ||
                                    m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount() >= m_RBStartCount + m_CountsToMove
                    ) {
                        this.stopTask();
                    }
                } else if (this.m_CountsToMove < 0) {
                    if (
                            m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() >= m_LTStartCount - m_CountsToMove ||
                                    m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount() >= m_RTStartCount - m_CountsToMove ||
                                    m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount() <= m_LBStartCount + m_CountsToMove ||
                                    m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount() <= m_RBStartCount + m_CountsToMove
                    ) {
                        this.stopTask();
                    }
                } else {
                    this.stopTask();
                }
            }

        }
    }
    public static class FixedZDistanceTask extends RobotMotionSystemFixedZDistanceTask{
        private OmniDrive m_Drive;
        private int m_CountsToMove = 0;
        private int m_LTStartCount = 0;
        private int m_RTStartCount = 0;
        private int m_LBStartCount = 0;
        private int m_RBStartCount = 0;
        private float m_StartDeg = 0;
        public FixedZDistanceTask(OmniDrive ODrive, double ZDistance, double Speed) {
            super(ZDistance, Speed);
            this.m_Drive = ODrive;
        }

        public FixedZDistanceTask(FixedZDistanceTask Task) {
            super(Task);
            this.m_Drive = Task.m_Drive;
        }

        @Override
        protected void __startTask() {
            __recalculateCounts();

            if(GlobalUtil.getGyro() != null){
                GlobalUtil.getGyro().updateStatus();
                m_StartDeg = m_Drive.isGyroGuidedDrivePublicStartingAngleEnabled() ? m_Drive.getGyroGuidedDrivePublicStartingAngle() : XYPlaneCalculations.normalizeDeg(GlobalUtil.getGyro().getHeading());
            }

            double AbsSpeed = Math.abs(getSpeed());
            if(m_Drive.isSteadySpeedUp() && AbsSpeed > this.getMotionSystem().getSteadySpeedUpThreshold()){
                AbsSpeed = 0;
            }
            double LTSpeed = this.m_CountsToMove > 0 ? -AbsSpeed : AbsSpeed;
            double RTSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;
            double LBSpeed = this.m_CountsToMove > 0 ? -AbsSpeed : AbsSpeed;
            double RBSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;

            m_LTStartCount = m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount();
            m_RTStartCount = m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount();
            m_LBStartCount = m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount();
            m_RBStartCount = m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount();


            m_Drive.m_LeftTopMotor.getMotorController().getMotor().setPower(LTSpeed);
            m_Drive.m_RightTopMotor.getMotorController().getMotor().setPower(RTSpeed);
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().setPower(LBSpeed);
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().setPower(RBSpeed);

            m_Drive.m_LeftTopMotor.getMotorController().getMotor().setCurrentMovingType(RobotMotor.MovingType.withSpeed);
            m_Drive.m_RightTopMotor.getMotorController().getMotor().setCurrentMovingType(RobotMotor.MovingType.withSpeed);
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().setCurrentMovingType(RobotMotor.MovingType.withSpeed);
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().setCurrentMovingType(RobotMotor.MovingType.withSpeed);
        }

        private void __recalculateCounts(){
            int CountToMove = (int) Math.round(this.getZDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getZPerCounterClockwiseDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.getLinearMotionDistanceFactor());
            CountToMove = Math.abs(CountToMove);
            int CountToMoveForward = (this.getZDistance() >= 0 ? CountToMove : -CountToMove);
            this.m_CountsToMove = CountToMoveForward;
        }

        @Override
        protected void __taskFinished() {
            if(this.m_Drive.getPositionTracker() != null) {
                int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;
                double DistanceMoved = CountsMoved / m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getRobotWheel().getZPerCounterClockwiseDistance() / m_Drive.getLinearMotionDistanceFactor();
                getMotionSystem().getPositionTracker().drive_MoveThroughRobotAngle(
                        0,
                        DistanceMoved
                );
            }
        }

        @Override
        public void updateStatus() {
            m_Drive.m_LeftTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().updateStatus();

            //Left / Right Speed Control
            if(this.isBusy()){
                double AbsSpeed = Math.abs(getSpeed());

                if(this.getMotionSystem().isSteadySpeedUp() && AbsSpeed > this.getMotionSystem().getSteadySpeedUpThreshold()){
                    double ExtraSpeed = AbsSpeed - this.getMotionSystem().getSteadySpeedUpThreshold();

                    int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;

                    double jobPercentile = Math.abs( ((double) CountsMoved) / m_CountsToMove);

                    if(jobPercentile < 0.30){
                        AbsSpeed = this.getMotionSystem().getSteadySpeedUpThreshold() + (jobPercentile / 0.30) * ExtraSpeed;
                    }else if(jobPercentile > 0.70){
                        AbsSpeed = this.getMotionSystem().getSteadySpeedUpThreshold() + ((1.0 - jobPercentile) / 0.30) * ExtraSpeed;
                    }
                }

                double LTSpeed = this.m_CountsToMove > 0 ? -AbsSpeed : AbsSpeed;
                double RTSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;
                double LBSpeed = this.m_CountsToMove > 0 ? -AbsSpeed : AbsSpeed;
                double RBSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;

                if(GlobalUtil.getGyro() != null && m_Drive.isGyroGuidedDriveEnabled()) {
                    GlobalUtil.getGyro().updateStatus();
                    double currentAng = GlobalUtil.getGyro().getHeading();
                    double deltaAng = XYPlaneCalculations.normalizeDeg(currentAng - m_StartDeg);
                    if (GlobalUtil.getGyro().getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.Clockwise) {
                        deltaAng = -deltaAng;
                    }


                    double deltaSpeedEachSide = 0;
                    if (Math.abs(deltaAng) >= 5) {
                        deltaSpeedEachSide = Range.clip(0.5 * AbsSpeed, 0, 0.25);
                    } else if (Math.abs(deltaAng) >= 3) {
                        deltaSpeedEachSide = Range.clip(0.25 * AbsSpeed, 0, 0.2);
                    } else if (Math.abs(deltaAng) >= 1) {
                        deltaSpeedEachSide = Range.clip(0.1 * AbsSpeed, 0, 0.1);
                    } else if (Math.abs(deltaAng) >= 0.5) {
                        deltaSpeedEachSide = Range.clip(0.05 * AbsSpeed, 0, 0.05);
                    }
                    if (deltaSpeedEachSide < 0.025 && deltaSpeedEachSide != 0) {
                        deltaSpeedEachSide = 0.025;
                    }


                    if (deltaAng > 0) {
                        LTSpeed -= deltaSpeedEachSide;
                        RTSpeed -= deltaSpeedEachSide;
                        LBSpeed -= deltaSpeedEachSide;
                        RBSpeed -= deltaSpeedEachSide;
                    } else if (deltaAng < 0) {
                        LTSpeed += deltaSpeedEachSide;
                        RTSpeed += deltaSpeedEachSide;
                        LBSpeed += deltaSpeedEachSide;
                        RBSpeed += deltaSpeedEachSide;
                    }
                }
                m_Drive.m_LeftTopMotor.getMotorController().getMotor().setPower(LTSpeed);
                m_Drive.m_RightTopMotor.getMotorController().getMotor().setPower(RTSpeed);
                m_Drive.m_LeftBottomMotor.getMotorController().getMotor().setPower(LBSpeed);
                m_Drive.m_RightBottomMotor.getMotorController().getMotor().setPower(RBSpeed);
            }

            //Distance / Count Control
            if(this.isBusy()){
            if (this.m_CountsToMove > 0) {
                if (
                        m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() <= m_LTStartCount - m_CountsToMove ||
                                m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount() >= m_RTStartCount + m_CountsToMove ||
                                m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount() <= m_LBStartCount - m_CountsToMove ||
                                m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount() >= m_RBStartCount + m_CountsToMove
                ) {
                    this.stopTask();
                }
            } else if (this.m_CountsToMove < 0) {
                if (
                        m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() >= m_LTStartCount - m_CountsToMove ||
                                m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount() <= m_RTStartCount + m_CountsToMove ||
                                m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount() >= m_LBStartCount - m_CountsToMove ||
                                m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount() <= m_RBStartCount + m_CountsToMove
                ) {
                    this.stopTask();
                }
            } else {
                this.stopTask();
            }
        }
        }
    }
    public static class FixedTurnTask extends RobotMotionSystemFixedTurnTask{
        private OmniDrive m_Drive;
        public FixedTurnTask(OmniDrive ODrive, double TurnDeg, double Speed) {
            super(TurnDeg, Speed);
            this.m_Drive = ODrive;
        }

        public FixedTurnTask(FixedTurnTask Task) {
            super(Task);
            this.m_Drive = Task.m_Drive;
        }

        @Override
        protected void __startTask() {
            int CountToMove = (int) Math.round(this.getTurnDeg() / 360.0 * 2.0 * Math.PI * this.m_Drive.m_LeftTopMotor.getRobotWheel().getDistanceFromCenterOfRobot() / this.m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.getRotationalMotionDistanceFactor());
            CountToMove = Math.abs(CountToMove);
            int CountToMoveCounterClockwise = (this.getTurnDeg() >= 0 ? CountToMove : -CountToMove);
            RobotMotorTaskCallBack FLCB = (this.getMotionSystem().getPositionTracker() == null ? null :
                    new RobotMotorTaskCallBack() {
                        @Override
                        public void finishRunning(RobotMotorController Controller, boolean timeOut, double timeUsedInSec, int CountsMoved) {
                            double DistanceMoved = CountsMoved / m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() / m_Drive.getRotationalMotionDistanceFactor();
                            getMotionSystem().getPositionTracker().drive_RotateAroundRobotPointWithRadiusAndPowerPoint(
                                    new Robot2DPositionIndicator(0,0,0),
                                    m_Drive.m_LeftTopMotor.getRobotWheel().getDistanceFromCenterOfRobot(),
                                    DistanceMoved
                            );
                        }
                    });

            RobotFixCountSpeedCtlTask LTTask = new RobotFixCountSpeedCtlTask(CountToMoveCounterClockwise,this.getSpeed(),FLCB,true);
            RobotFixCountSpeedCtlTask RTTask = new RobotFixCountSpeedCtlTask(CountToMoveCounterClockwise,this.getSpeed(),null,true);
            RobotFixCountSpeedCtlTask LBTask = new RobotFixCountSpeedCtlTask(CountToMoveCounterClockwise,this.getSpeed(),null,true);
            RobotFixCountSpeedCtlTask RBTask = new RobotFixCountSpeedCtlTask(CountToMoveCounterClockwise,this.getSpeed(),null,true);
            m_Drive.m_LeftTopMotor.getMotorController().replaceTask(LTTask);
            m_Drive.m_RightTopMotor.getMotorController().replaceTask(RTTask);
            m_Drive.m_LeftBottomMotor.getMotorController().replaceTask(LBTask);
            m_Drive.m_RightBottomMotor.getMotorController().replaceTask(RBTask);
        }

        @Override
        protected void __taskFinished() {

        }

        @Override
        public void updateStatus() {
            m_Drive.m_LeftTopMotor.getMotorController().updateStatus();
            m_Drive.m_RightTopMotor.getMotorController().updateStatus();
            m_Drive.m_LeftBottomMotor.getMotorController().updateStatus();
            m_Drive.m_RightBottomMotor.getMotorController().updateStatus();
            if(!(m_Drive.m_LeftTopMotor.getMotorController().isBusy() || m_Drive.m_LeftBottomMotor.getMotorController().isBusy() || m_Drive.m_RightTopMotor.getMotorController().isBusy() || m_Drive.m_RightBottomMotor.getMotorController().isBusy())){
                this.stopTask();
            }
        }
    }
    public static class TeleOpControlTask extends RobotMotionSystemTeleOpControlTask{
        private OmniDrive m_Drive;
        protected RobotFixedSpeedTask m_FLTask = null;
        protected RobotFixedSpeedTask m_FRTask = null;
        protected RobotFixedSpeedTask m_BLTask = null;
        protected RobotFixedSpeedTask m_BRTask = null;

        public TeleOpControlTask(OmniDrive ODrive){
            this.m_Drive = ODrive;
        }

        public TeleOpControlTask(TeleOpControlTask Task){
            super(Task);
            this.m_Drive = Task.m_Drive;
        }

        @Override
        protected void __updateDriveSpeedAndPositionTracker() {
            double FLPower = -super.getDriveXSpeed() - super.getDriveZSpeed() + super.getDriveRotationSpeed();
            double FRPower = -super.getDriveXSpeed() + super.getDriveZSpeed() + super.getDriveRotationSpeed();
            double BLPower = super.getDriveXSpeed() - super.getDriveZSpeed() + super.getDriveRotationSpeed();
            double BRPower = super.getDriveXSpeed() + super.getDriveZSpeed() + super.getDriveRotationSpeed();
            FLPower = Range.clip(FLPower,-1.0,1.0);
            FRPower = Range.clip(FRPower,-1.0,1.0);
            BLPower = Range.clip(BLPower,-1.0,1.0);
            BRPower = Range.clip(BRPower,-1.0,1.0);
            this.m_FLTask.setSpeed(FLPower);
            this.m_FRTask.setSpeed(FRPower);
            this.m_BLTask.setSpeed(BLPower);
            this.m_BRTask.setSpeed(BRPower);
        }

        @Override
        protected void __startDrive() {
            this.m_FLTask = new RobotFixedSpeedTask(0,0,null);
            this.m_FRTask = new RobotFixedSpeedTask(0,0,null);
            this.m_BLTask = new RobotFixedSpeedTask(0,0,null);
            this.m_BRTask = new RobotFixedSpeedTask(0,0,null);
            m_Drive.m_LeftTopMotor.getMotorController().replaceTask(m_FLTask);
            m_Drive.m_RightTopMotor.getMotorController().replaceTask(m_FRTask);
            m_Drive.m_LeftBottomMotor.getMotorController().replaceTask(m_BLTask);
            m_Drive.m_RightBottomMotor.getMotorController().replaceTask(m_BRTask);
            this.__updateDriveSpeedAndPositionTracker();
        }

        @Override
        protected void __updateMotorStatus() {
            this.m_Drive.m_LeftTopMotor.getMotorController().updateStatus();
            this.m_Drive.m_LeftBottomMotor.getMotorController().updateStatus();
            this.m_Drive.m_RightTopMotor.getMotorController().updateStatus();
            this.m_Drive.m_RightBottomMotor.getMotorController().updateStatus();
        }

        @Override
        protected void __taskFinished() {

        }
    }

    private RobotMotion m_LeftTopMotor, m_RightTopMotor, m_LeftBottomMotor, m_RightBottomMotor;

    public OmniDrive(RobotMotion LeftTopMotor, RobotMotion RightTopMotor, RobotMotion LeftBottomMotor, RobotMotion RightBottomMotor, Robot2DPositionTracker PositionTracker) {
        super(PositionTracker);
        this.m_LeftTopMotor = LeftTopMotor;
        this.m_RightTopMotor = RightTopMotor;
        this.m_LeftBottomMotor = LeftBottomMotor;
        this.m_RightBottomMotor = RightBottomMotor;
    }

    public OmniDrive(OmniDrive MotionSystem) {
        super(MotionSystem);
        this.m_LeftTopMotor = MotionSystem.m_LeftTopMotor;
        this.m_RightTopMotor = MotionSystem.m_RightTopMotor;
        this.m_LeftBottomMotor = MotionSystem.m_LeftBottomMotor;
        this.m_RightBottomMotor = MotionSystem.m_RightBottomMotor;
    }

    @Override
    protected void __stopMotion() {
        this.m_LeftTopMotor.getMotorController().getMotor().setPower(0);
        this.m_RightTopMotor.getMotorController().getMotor().setPower(0);
        this.m_LeftBottomMotor.getMotorController().getMotor().setPower(0);
        this.m_RightBottomMotor.getMotorController().getMotor().setPower(0);
        this.m_LeftTopMotor.getMotorController().deleteAllTasks();
        this.m_RightTopMotor.getMotorController().deleteAllTasks();
        this.m_LeftBottomMotor.getMotorController().deleteAllTasks();
        this.m_RightBottomMotor.getMotorController().deleteAllTasks();
    }

    @Override
    public RobotMotionSystemFixedXDistanceTask getFixedXDistanceTask(double XDistance, double Speed) {
        return new FixedXDistanceTask(this,XDistance,Speed);
    }

    @Override
    public RobotMotionSystemFixedZDistanceTask getFixedZDistanceTask(double ZDistance, double Speed) {
        return new FixedZDistanceTask(this,ZDistance,Speed);
    }

    @Override
    public RobotMotionSystemFixedTurnTask getFixedTurnTask(double Deg, double Speed) {
        return new FixedTurnTask(this,Deg,Speed);
    }

    @Override
    public RobotMotionSystemTeleOpControlTask getTeleOpTask() {
        return new TeleOpControlTask(this);
    }
}
