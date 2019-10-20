package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.HashMap;

public class Robot implements AutoCloseable {

    public final ElapsedTime runtime;

    public final DriveTrain driveTrain;
    public final FoundationGrabber foundationGrabber;
    public final Lift lift;
    public final StoneManipulator stoneManipulator; // includes capstone
    public final Camera camera;
    public final Phone phone;
    public HashMap<String, ExpansionHub> expansionHubs;

    private static Robot instance = null;

    public static synchronized Robot getInstance() {
        return instance != null ? instance : (instance = new Robot());
    }

    private Robot() {

        runtime = new ElapsedTime();

        driveTrain = DriveTrain.getInstance();
        foundationGrabber = FoundationGrabber.getInstance();
        lift = Lift.getInstance();
        stoneManipulator = StoneManipulator.getInstance();
        camera = Camera.getInstance();
        phone = Phone.getInstance();

    }

    public void init(HardwareMap hardwareMap) {
        driveTrain.init(hardwareMap);
        foundationGrabber.init(hardwareMap);
        lift.init(hardwareMap);
        stoneManipulator.init(hardwareMap);
        camera.init(hardwareMap);
//        phone.init(hardwareMap);
        expansionHubs = ExpansionHub.getAvailableHubs(hardwareMap);
    }

    @Override
    public void close() {
        camera.stop();
        phone.stopGyro();
        phone.stopTextToSpeech();
    }
}
