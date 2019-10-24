package org.firstinspires.ftc.teamcode.constants

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.MotorConfig
import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.ServoConfig
import com.qualcomm.robotcore.hardware.DcMotorSimple as Motor
import org.firstinspires.ftc.teamcode.MOEStuff.MOEBot.OdometryWheelConfig as OWC

object MOEConstants {


    object DriveTrain {
        object Motors {
            object Configs {
                val FrontLeft = MotorConfig("FLDM11")
                val FrontRight = MotorConfig("FRDM13", Motor.Direction.REVERSE)
                val BackLeft = MotorConfig("BLDM10")
                val BackRight = MotorConfig("BRDM12", Motor.Direction.REVERSE)
            }
        }
    }

    object Odometry {
        object Servos {
            object Configs {
                val Left = ServoConfig("LODS10", 0.0, 0.45, Servo.Direction.REVERSE)
                val Right = ServoConfig("RODS11", 0.55, 1.0, Servo.Direction.FORWARD)
            }
        }

        object Wheels {
            object Configs {
                val axialLeft = OWC("LOAA12", OWC.Direction.FORWARD, OWC.Orientation.AXIAL)
                val axialRight = OWC("ROAA10", OWC.Direction.FORWARD, OWC.Orientation.AXIAL)
                val strafeRight = OWC("ROSA11", OWC.Direction.FORWARD, OWC.Orientation.STRAFE)
            }

            object Circumference {
                val AXIAL = 0.0 //TODO: Calculate.

                val STRAFE = 0.0  //TODO: Calculate.
            }


        }
    }
}
