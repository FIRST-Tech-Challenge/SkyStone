package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap

interface Factory<Product> {
    fun standard(hardwareMap: HardwareMap): Product
}