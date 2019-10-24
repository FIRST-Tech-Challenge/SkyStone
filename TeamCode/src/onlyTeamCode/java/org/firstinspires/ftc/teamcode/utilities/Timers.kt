package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.MOEStuff.MOEOpmodes.MOEOpMode
import java.util.concurrent.TimeUnit


infix fun MOEOpMode.wait(milliseconds: Number) {
    val time = ElapsedTime();
    val length = milliseconds.toLong()
    while (time.milliseconds() < length && opModeIsActive());
}

