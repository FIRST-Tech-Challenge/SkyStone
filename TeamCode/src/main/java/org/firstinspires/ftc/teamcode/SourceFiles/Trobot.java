/**
 * Copyright (c) 2020, All Rights Reserved
 *
 * Trobot is a composition of 'Drivetrain' and 'Component'. It is the master file that each OpMode,
 * whether TeleOp or autonomous, will implement. While it doesn't have any utility methods, it
 * improves code concision by combining different parts into a single object that the user can
 * import and implement.
 *
 * Written by Timothy (Tikki) Cui
 */


package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Trobot {
    private HardwareMap hardwareMap;

    private Drivetrain drivetrain;
    private Component component;

    private ElapsedTime runtime;

    public Trobot() {
        // Constructor must utilize a Hardware Map from the source. However, Java always automatically
        // creates a default constructor, so custom error message must be made to catch error
        throw new NullPointerException("Must pass hardwareMap into constructor");
    }

    public Trobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
        component = new Component(hardwareMap);

        runtime = new ElapsedTime();
    }

    // Accessor/Mutator
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public Drivetrain getDrivetrain() {return drivetrain;}
    public Component getComponent() {return component;}
    public ElapsedTime getRuntime() {return runtime;}

    public void setHardwareMap(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap; }
    public void setDrivetrain(Drivetrain drivetrain) {this.drivetrain = drivetrain;}
    public void setComponent(Component component) {this.component = component;}
    public void setElapsedTime(ElapsedTime runtime) {this.runtime = runtime;}
}