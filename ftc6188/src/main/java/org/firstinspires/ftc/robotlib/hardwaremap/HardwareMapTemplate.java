package org.firstinspires.ftc.robotlib.hardwaremap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.controller.Controller;
import org.firstinspires.ftc.robotlib.drivetrain.Drivetrain;

public abstract class HardwareMapTemplate
{
    DcMotor[] motorList;
    Drivetrain drivetrain;
    Controller controller;
}
