package org.firstinspires.ftc.teamcode.OtherStuff.MOEBotStuff

import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.MOEServo
import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.MOEdometryWheel
import org.firstinspires.ftc.teamcode.constants.MOEConstants.Odometry
import org.firstinspires.ftc.teamcode.constants.ReferenceHolder.Companion.moeOpMode
import org.firstinspires.ftc.teamcode.utilities.wait

class MOEdometrySystem {
    val servos = ServosHolder()

    class ServosHolder {
        val left: MOEServo = MOEServo(Odometry.Servos.Configs.Left)
        val right: MOEServo = MOEServo(Odometry.Servos.Configs.Right)

        fun initServosUp() {
            left.setPosition(0.0)
            right.setPosition(0.0)
        }

        fun initServosDown() {
            initServosUp()
            moeOpMode.wait(2000)
            left.setPositionOverTime(1.0, 1.0, async = true)
            right.setPositionOverTime(1.0, 1.0, async = true)
        }
    }

    class WheelsHolder {
        val axialRight = MOEdometryWheel(Odometry.Wheels.Names.axialRight, Odometry.Wheels.Configs.axialRight)
        val axialLeft = MOEdometryWheel(Odometry.Wheels.Names.axialLeft, Odometry.Wheels.Configs.axialLeft)
        val strafeRight = MOEdometryWheel(Odometry.Wheels.Names.strafeRight, Odometry.Wheels.Configs.strafeRight)
    }

//    var x: AtomicInteger
//    var y: AtomicInteger
//    var totalFront: AtomicInteger
//    var totalSide: AtomicInteger
//    //    public AtomicInteger debugInt;
//    var right: MOEdometryWheel
//    var front: MOEdometryWheel
//    private var overallX: Double = 0.toDouble()
//    private var overallY: Double = 0.toDouble()
//    private var xOffset: Double = 0.toDouble()
//    private var yOffset: Double = 0.toDouble()
//    //
//    //    public double getExtra() {
//    //        return ((double) debugInt.get()) / odometryScale;
//    //    }
//    //
//    //    public void setExtra(double value) {
//    //        debugInt.set((int) Math.round(value * odometryScale));
//    //    }
//
//    val oldOdometryPosition: Point
//        get() = Point(oldGetOdometryX(), oldGetOdometryY())
//
//    val odometryPosition: Point
//        get() = Point(getX(), getY())
//
//    var odometryTotals: DoubleArray
//        get() =
//            doubleArrayOf(this.totalFront.get().toDouble() / odometryScale, this.totalSide.get().toDouble() / odometryScale)
//        set(totals) {
//            this.totalFront.set((totals[0] * odometryScale) as Int)
//            this.totalSide.set((totals[1] * odometryScale) as Int)
//        }
//
//
//    val position: Point
//        get() = Point(getX(), getY())
//
//    init {
//        this.front = MOEdometryWheel("frontOdometry")
//        //BEFORE: -19.83125, -17.30625, -15.85625
//        // 5.05, 1.25
//        //1.6,1.2
//        //        this.prefs = prefs;
//        this.right = MOEdometryWheel(hardMapRef.get(AnalogInput::class.java, "rightOdometry"),
//                telemetry, gyro, false, prefs.getSideOdometryTurnCorrection(), prefs)
//        //BEFORE: -10.544444, -9.294444, -8.294444
//        x = AtomicInteger(0)
//        y = AtomicInteger(0)
//        this.ODOMETRY_VOLTS_PER_INCH = prefs.getFrontOdometryTics()
//        this.ODOMETRY_ASTARUNITS_PER_INCH = this.ODOMETRY_VOLTS_PER_INCH * 2
//        this.overallX = 0.0
//        this.overallY = 0.0
//        this.totalFront = AtomicInteger(0)
//        this.totalSide = AtomicInteger(0)
//    }
//
//    fun oldGetOdometryX(): Double {
//        return x.get().toDouble() / odometryScale * ODOMETRY_ASTARUNITS_PER_INCH
//    }
//
//    fun oldGetOdometryY(): Double {
//        return y.get().toDouble() / odometryScale * ODOMETRY_ASTARUNITS_PER_INCH
//    }
//
//    fun updateXY(frontOps: MOEdometry, sideOps: MOEdometry) {
//        val frontAngle = gyro.getAutonAngle()
//        val rightAngle = gyro.addAngles(frontAngle, -90)
//        val frontHeading = MOEVector(Math.cos(Math.toRadians(frontAngle)) * frontOps.getOdometryChange(), Math.sin(Math.toRadians(frontAngle)) * frontOps.getOdometryChange())
//        val rightHeading = MOEVector(Math.cos(Math.toRadians(rightAngle)) * sideOps.getOdometryChange(), Math.sin(Math.toRadians(rightAngle)) * sideOps.getOdometryChange())
//        val overallHeading = MOEVector.addVectors(frontHeading, rightHeading)
//        overallX += overallHeading.getX()
//        overallY += overallHeading.getY()
//        // overallX += curFrontTics;
//        // overallX += frontOps;
//        // overallY += sideOps;
//        setX(overallX)
//        setY(overallY)
//        // telemetry.addData("front", Math.round(curFrontTics*100));
//        // telemetry.addData("right", Math.round(curRightTics*100));
//        // telemetry.addData("front", Math.round(frontOps*100));
//        // telemetry.addData("right", Math.round(sideOps*100));
//        // telemetry.update();
//    }
//
//    fun startOdometryThread(telemetry: Telemetry, gyro: MOEGyro) {
//        val bo = Thread { runLoop(opMode, telemetry, gyro) }
//        bo.priority = Thread.MAX_PRIORITY
//        front.setPresets(gyro)
//        right.setPresets(gyro)
//        bo.start()
//    }
//
//    fun setX(value: Double) {
//        x.set(Math.round(value * odometryScale) as Int)
//    }
//
//    fun setY(value: Double) {
//        y.set(Math.round(value * odometryScale) as Int)
//    }
//
//    private fun runLoop() {
//        while (moeOpMode.opModeIsActive()) {
//            val currentAngle = robot.gyro.getAutonAngle()
//            front.updateOdometry( currentAngle)
//            right.updateOdometry(opMode, currentAngle)
//            updateXY(front, right)
//
//            opMode.robot.prefs.addLogs(overallX.toString() + "\t" + overallY)
//
//            //            setOdometryTotals(new double[] {front.getOdometryValue(), right.getOdometryValue()});
//            //                        setX(front.getOdometryValue());
//            //                        setY(right.getOdometryValue());
//            //            setExtra(front.getOdometryChange());
//        }
//    }
//
//    fun reset() {
//        setTo(0.0, 0.0)
//    }
//
//    fun getX(): Double {
//        return oldGetOdometryX() + xOffset
//    }
//
//    fun getY(): Double {
//        return oldGetOdometryY() + yOffset
//    }
//
//    fun setTo(x: Double, y: Double) {
//        xOffset = x - oldGetOdometryX()
//        yOffset = y - oldGetOdometryY()
//    }
}