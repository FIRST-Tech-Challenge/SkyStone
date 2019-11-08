package org.darbots.darbotsftclib.libcore.chassiscontrollers;

import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.tasks.motor_tasks.RobotFixedSpeedTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedTurnTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedXDistanceTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedZDistanceTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTeleOpControlTask;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionSoftwareTracker;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;

public class OmniDrive extends RobotMotionSystem {
    public static class FixedXDistanceTask extends RobotMotionSystemFixedXDistanceTask{
        private OmniDrive m_Drive;
        private int m_CountsToMove = 0;
        private int m_LTStartCount = 0;
        private int m_RTStartCount = 0;
        private int m_LBStartCount = 0;
        private int m_RBStartCount = 0;

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

            m_LTStartCount = m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount();
            m_RTStartCount = m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount();
            m_LBStartCount = m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount();
            m_RBStartCount = m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount();

            double AbsSpeed = getSpeed();
            AbsSpeed = super.__getSupposedSteadilySpeedUpAbsSpeed(AbsSpeed);
            double xSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;

            double LTSpeed = -xSpeed;
            double RTSpeed = -xSpeed;
            double LBSpeed = xSpeed;
            double RBSpeed = xSpeed;


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
            int CountToMove = (int) Math.round(this.getXDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getXPerCounterClockwiseDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.getLinearXMotionDistanceFactor());
            CountToMove = Math.abs(CountToMove);
            int CountToMoveRight = (this.getXDistance() >= 0 ? CountToMove : -CountToMove);
            this.m_CountsToMove = CountToMoveRight;
        }

        @Override
        protected void __taskFinished() {
            if(this.m_Drive.getPositionTracker() != null) {
                if(this.m_Drive.getPositionTracker() instanceof Robot2DPositionSoftwareTracker) {
                    Robot2DPositionSoftwareTracker softwareTracker = (Robot2DPositionSoftwareTracker) this.m_Drive.getPositionTracker();
                    int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;
                    double DistanceMoved = CountsMoved / m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getRobotWheel().getXPerCounterClockwiseDistance() / m_Drive.getLinearXMotionDistanceFactor();
                    softwareTracker.drive_MoveThroughRobotAngle(
                            -90,
                            DistanceMoved
                    );
                }
            }
        }

        @Override
        public double getTaskProgressRatio() {
            int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;
            double jobPercentile = ((double) -CountsMoved) / m_CountsToMove;
            return jobPercentile;
        }

        @Override
        public void updateStatus() {
            m_Drive.m_LeftTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().updateStatus();
            //Left / Right Speed Control
            if(this.isBusy()){
                double AbsSpeed = getSpeed();
                AbsSpeed = super.__getSupposedSteadilySpeedUpAbsSpeed(AbsSpeed);
                double xSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;

                double LTSpeed = -xSpeed;
                double RTSpeed = -xSpeed;
                double LBSpeed = xSpeed;
                double RBSpeed = xSpeed;

                double GyroDelta = super.__getGyroGuidedDeltaSpeed(AbsSpeed);

                LTSpeed += GyroDelta;
                RTSpeed += GyroDelta;
                LBSpeed += GyroDelta;
                RBSpeed += GyroDelta;

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

            m_LTStartCount = m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount();
            m_RTStartCount = m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount();
            m_LBStartCount = m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount();
            m_RBStartCount = m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount();

            double AbsSpeed = getSpeed();
            AbsSpeed = super.__getSupposedSteadilySpeedUpAbsSpeed(AbsSpeed);
            double zSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;

            double LTSpeed = -zSpeed;
            double RTSpeed = zSpeed;
            double LBSpeed = -zSpeed;
            double RBSpeed = zSpeed;

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
            int CountToMove = (int) Math.round(this.getZDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getZPerCounterClockwiseDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.getLinearZMotionDistanceFactor());
            CountToMove = Math.abs(CountToMove);
            int CountToMoveForward = (this.getZDistance() >= 0 ? CountToMove : -CountToMove);
            this.m_CountsToMove = CountToMoveForward;
        }

        @Override
        protected void __taskFinished() {
            if(this.m_Drive.getPositionTracker() != null) {
                if(this.m_Drive.getPositionTracker() instanceof Robot2DPositionSoftwareTracker) {
                    Robot2DPositionSoftwareTracker softwareTracker = (Robot2DPositionSoftwareTracker) this.m_Drive.getPositionTracker();
                    int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;
                    double DistanceMoved = CountsMoved / m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getRobotWheel().getZPerCounterClockwiseDistance() / m_Drive.getLinearZMotionDistanceFactor();
                    softwareTracker.drive_MoveThroughRobotAngle(
                            0,
                            DistanceMoved
                    );
                }
            }
        }

        @Override
        public double getTaskProgressRatio() {
            int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;
            double jobPercentile = ((double) -CountsMoved) / m_CountsToMove;
            return jobPercentile;
        }

        @Override
        public void updateStatus() {
            m_Drive.m_LeftTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().updateStatus();

            //Left / Right Speed Control
            if(this.isBusy()){
                double AbsSpeed = getSpeed();

                AbsSpeed = super.__getSupposedSteadilySpeedUpAbsSpeed(AbsSpeed);
                double zSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;

                double LTSpeed = -zSpeed;
                double RTSpeed = zSpeed;
                double LBSpeed = -zSpeed;
                double RBSpeed = zSpeed;

                double GyroDelta = super.__getGyroGuidedDeltaSpeed(AbsSpeed);

                LTSpeed += GyroDelta;
                RTSpeed += GyroDelta;
                LBSpeed += GyroDelta;
                RBSpeed += GyroDelta;

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
        private int m_CountsToMove = 0;
        private int m_LTStartCount = 0;
        private int m_RTStartCount = 0;
        private int m_LBStartCount = 0;
        private int m_RBStartCount = 0;

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
            __recalculateCounts();

            m_LTStartCount = m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount();
            m_RTStartCount = m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount();
            m_LBStartCount = m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount();
            m_RBStartCount = m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount();

            double AbsSpeed = getSpeed();
            AbsSpeed = super.__getSupposedSteadilySpeedUpAbsSpeed(AbsSpeed);
            double turnSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;
            double LTSpeed = turnSpeed;
            double RTSpeed = turnSpeed;
            double LBSpeed = turnSpeed;
            double RBSpeed = turnSpeed;

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
            int CountToMove = (int) Math.round(this.getTurnDeg() / 360.0 * 2.0 * Math.PI * this.m_Drive.m_LeftTopMotor.getRobotWheel().getDistanceFromCenterOfRobot() / this.m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.getRotationalMotionDistanceFactor());
            CountToMove = Math.abs(CountToMove);
            int CountToMoveForward = (this.getTurnDeg() >= 0 ? CountToMove : -CountToMove);
            this.m_CountsToMove = CountToMoveForward;
        }

        @Override
        protected void __taskFinished() {
            if(this.m_Drive.getPositionTracker() != null) {
                if(this.m_Drive.getPositionTracker() instanceof Robot2DPositionSoftwareTracker) {
                    Robot2DPositionSoftwareTracker softwareTracker = (Robot2DPositionSoftwareTracker) this.m_Drive.getPositionTracker();
                    int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;
                    double DistanceMoved = CountsMoved / m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() / m_Drive.getRotationalMotionDistanceFactor();
                    softwareTracker.drive_RotateAroundRobotOriginWithRadius(
                            m_Drive.m_LeftTopMotor.getRobotWheel().getDistanceFromCenterOfRobot(),
                            DistanceMoved
                    );
                }
            }
        }

        @Override
        public double getTaskProgressRatio() {
            int CountsMoved = this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() - this.m_LTStartCount;
            double jobPercentile = Math.abs( ((double) CountsMoved) / m_CountsToMove);
            return jobPercentile;
        }

        @Override
        public void updateStatus() {
            m_Drive.m_LeftTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightTopMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_LeftBottomMotor.getMotorController().getMotor().updateStatus();
            m_Drive.m_RightBottomMotor.getMotorController().getMotor().updateStatus();

            //Left / Right Speed Control
            if(this.isBusy()){
                double AbsSpeed = getSpeed();

                AbsSpeed = super.__getSupposedSteadilySpeedUpAbsSpeed(AbsSpeed);
                double turnSpeed = this.m_CountsToMove > 0 ? AbsSpeed : -AbsSpeed;
                double LTSpeed = turnSpeed;
                double RTSpeed = turnSpeed;
                double LBSpeed = turnSpeed;
                double RBSpeed = turnSpeed;

                m_Drive.m_LeftTopMotor.getMotorController().getMotor().setPower(LTSpeed);
                m_Drive.m_RightTopMotor.getMotorController().getMotor().setPower(RTSpeed);
                m_Drive.m_LeftBottomMotor.getMotorController().getMotor().setPower(LBSpeed);
                m_Drive.m_RightBottomMotor.getMotorController().getMotor().setPower(RBSpeed);
            }

            //Distance / Count Control
            if(this.isBusy()){
                if (this.m_CountsToMove > 0) {
                    if (
                            m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() >= m_LTStartCount + m_CountsToMove ||
                                    m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount() >= m_RTStartCount + m_CountsToMove ||
                                    m_Drive.m_LeftBottomMotor.getMotorController().getMotor().getCurrentCount() >= m_LBStartCount + m_CountsToMove ||
                                    m_Drive.m_RightBottomMotor.getMotorController().getMotor().getCurrentCount() >= m_RBStartCount + m_CountsToMove
                    ) {
                        this.stopTask();
                    }
                } else if (this.m_CountsToMove < 0) {
                    if (
                            m_Drive.m_LeftTopMotor.getMotorController().getMotor().getCurrentCount() <= m_LTStartCount + m_CountsToMove ||
                                    m_Drive.m_RightTopMotor.getMotorController().getMotor().getCurrentCount() <= m_RTStartCount + m_CountsToMove ||
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
