package org.darbots.darbotsftclib.libcore.chassiscontrollers;

import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionTracker;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.tasks.motor_tasks.RobotFixCountSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.tasks.motor_tasks.RobotFixedSpeedTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedTurnTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedXDistanceTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemFixedZDistanceTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTeleOpControlTask;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotorTaskCallBack;

public class OmniDrive extends RobotMotionSystem {
    public static class FixedXDistanceTask extends RobotMotionSystemFixedXDistanceTask{
        private OmniDrive m_Drive;
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
            int CountToMove = (int) Math.round(this.getXDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getXPerCounterClockwiseDistance() / m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.getLinearMotionDistanceFactor());
            CountToMove = Math.abs(CountToMove);
            int CountToMoveRight = (this.getXDistance() >= 0 ? CountToMove : -CountToMove);
            RobotMotorTaskCallBack FLCB = (this.getMotionSystem().getPositionTracker() == null ? null :
                new RobotMotorTaskCallBack() {
                    @Override
                    public void finishRunning(RobotMotorController Controller, boolean timeOut, double timeUsedInSec, int CountsMoved) {
                        double DistanceMoved = CountsMoved / m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getRobotWheel().getXPerCounterClockwiseDistance() / m_Drive.getLinearMotionDistanceFactor();
                        getMotionSystem().getPositionTracker().drive_MoveThroughRobotAngle(
                                -90,
                                DistanceMoved
                                );
                    }
                });
            m_Drive.m_LeftTopMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(-CountToMoveRight, this.getSpeed(), FLCB , true));
            m_Drive.m_RightTopMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(-CountToMoveRight,this.getSpeed(),null,true));
            m_Drive.m_LeftBottomMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(CountToMoveRight,this.getSpeed(),null,true));
            m_Drive.m_RightBottomMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(CountToMoveRight,this.getSpeed(),null,true));
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
    public static class FixedZDistanceTask extends RobotMotionSystemFixedZDistanceTask{
        private OmniDrive m_Drive;
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
            int CountToMove = (int) Math.round(this.getZDistance() / this.m_Drive.m_LeftTopMotor.getRobotWheel().getZPerCounterClockwiseDistance() / this.m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * this.m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.getLinearMotionDistanceFactor());
            CountToMove = Math.abs(CountToMove);
            int CountToMoveForward = (this.getZDistance() >= 0 ? CountToMove : -CountToMove);
            RobotMotorTaskCallBack FLCB = (this.getMotionSystem().getPositionTracker() == null ? null :
                    new RobotMotorTaskCallBack() {
                        @Override
                        public void finishRunning(RobotMotorController Controller, boolean timeOut, double timeUsedInSec, int CountsMoved) {
                            double DistanceMoved = CountsMoved / m_Drive.m_LeftTopMotor.getMotorController().getMotor().getMotorType().getCountsPerRev() * m_Drive.m_LeftTopMotor.getRobotWheel().getCircumference() * m_Drive.m_LeftTopMotor.getRobotWheel().getZPerCounterClockwiseDistance() / m_Drive.getLinearMotionDistanceFactor();
                            getMotionSystem().getPositionTracker().drive_MoveThroughRobotAngle(
                                    0,
                                    DistanceMoved
                            );
                        }
                    });
            m_Drive.m_LeftTopMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(-CountToMoveForward,this.getSpeed(),FLCB,true));
            m_Drive.m_RightTopMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(CountToMoveForward,this.getSpeed(),null,true));
            m_Drive.m_LeftBottomMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(-CountToMoveForward,this.getSpeed(),null,true));
            m_Drive.m_RightBottomMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(CountToMoveForward,this.getSpeed(),null,true));
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
            m_Drive.m_LeftTopMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(CountToMoveCounterClockwise,this.getSpeed(),FLCB,true));
            m_Drive.m_RightTopMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(CountToMoveCounterClockwise,this.getSpeed(),null,true));
            m_Drive.m_LeftBottomMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(CountToMoveCounterClockwise,this.getSpeed(),null,true));
            m_Drive.m_RightBottomMotor.getMotorController().replaceTask(new RobotFixCountSpeedCtlTask(CountToMoveCounterClockwise,this.getSpeed(),null,true));
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
