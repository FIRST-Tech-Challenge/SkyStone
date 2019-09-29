package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.List;

public class Robot implements AutoCloseable {

    public final ElapsedTime runtime;

    public final DriveTrain driveTrain;
    public final FoundationGrabber foundationGrabber;
    public final Lift lift;
    public final StoneManipulator stoneManipulator; // includes capstone
    public final Camera camera;
    public final Phone phone;
    public List<ExpansionHub> expansionHubs;

    private static Robot instance = null;

    public static Robot getInstance(HardwareMap hardwareMap) {
        return instance != null ? instance : (instance = new Robot(hardwareMap));
    }

    private Robot(HardwareMap hardwareMap) {

        runtime = new ElapsedTime();

        driveTrain = DriveTrain.getInstance(hardwareMap);
        foundationGrabber = FoundationGrabber.getInstance(hardwareMap);
        lift = Lift.getInstance(hardwareMap);
        stoneManipulator = StoneManipulator.getInstance(hardwareMap);
        camera = Camera.getInstance(hardwareMap);
        phone = Phone.getInstance(hardwareMap);
        expansionHubs = ExpansionHub.getAvailableHubs(hardwareMap);

    }

    @Override
    public void close() {
        camera.stop();
        phone.stopGyro();
        phone.stopTextToSpeech();
    }
}
