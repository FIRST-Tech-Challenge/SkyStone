package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class DeviceMap {
    private static DeviceMap INSTANCE;
    private static Telemetry telemetry;

    private DcMotor leftTop, leftBottom, rightTop, rightBottom;

    public DeviceMap(HardwareMap map) {
        //for later
        ExecutorService service = Executors.newFixedThreadPool(5);
        setUpMotors(map);

        INSTANCE = this;
    }

    /**
     * This will just set up all the motors
     * @param map
     */
    private void setUpMotors(HardwareMap map) {
        telemetry.addLine("Setting up motors");
        telemetry.update();
        leftTop = map.get(DcMotor.class, "LeftTop");
        leftBottom = map.get(DcMotor.class, "LeftBottom");
        rightTop = map.get(DcMotor.class, "RightTop");
        rightBottom = map.get(DcMotor.class, "RightBottom");

        for(DcMotor motor : getMotors()) {
            motor.setPower(0);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        rightTop.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("Finished setting up motors");
        telemetry.update();
    }

    //The methods below get all the motors
    public DcMotor getLeftTop() {
        return leftTop;
    }
    public DcMotor getLeftBottom() {
        return leftBottom;
    }
    public DcMotor getRightTop() {
        return rightTop;
    }
    public DcMotor getRightBottom() {
        return rightBottom;
    }
    public DcMotor[] getMotors() {
        return new DcMotor[]{leftTop, rightTop, leftBottom, rightBottom};
    }

    public static DeviceMap getInstance() {
        if(INSTANCE == null) throw new RuntimeException("the constructor must be called first");
        return INSTANCE;
    }
    public static DeviceMap getInstance(HardwareMap map) {
        if(INSTANCE == null && map != null) INSTANCE = new DeviceMap(map);
        return INSTANCE;
    }
    public static void setTelemetry(Telemetry ttelemetry) {
        telemetry = ttelemetry;
    }
}
