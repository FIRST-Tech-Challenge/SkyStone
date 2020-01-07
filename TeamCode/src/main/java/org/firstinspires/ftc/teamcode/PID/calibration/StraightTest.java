package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "StraightTest", group = "drive")
@Disabled
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 72;
    private PIDCoefficients coefficients;
    private double kV;

    @Override
    public void runOpMode() {
        DriveConstantsPID.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = null;
        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        HardwareMap hwMap = new HardwareMap(hardwareMap);

        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        //drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.resetEncoders();
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();
        PIDCoefficients oldPID = new PIDCoefficients(DriveConstantsPID.MOTOR_VELO_PID.kP, DriveConstantsPID.MOTOR_VELO_PID.kI,
                DriveConstantsPID.MOTOR_VELO_PID.kD);
        double oldkV = DriveConstantsPID.kV;

        coefficients = new PIDCoefficients(DriveConstantsPID.MOTOR_VELO_PID.kP, DriveConstantsPID.MOTOR_VELO_PID.kI,
                DriveConstantsPID.MOTOR_VELO_PID.kD);
        kV = DriveConstantsPID.kV;


        int selected = 0;
        boolean blocker1 = false;
        boolean blocker2 = false;
        boolean blocker3 = false;

        while (!isStarted()) {
            if (gamepad1.left_stick_y >= 0.5) {
                DISTANCE -= 1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            } else if (gamepad1.left_stick_y <= -0.5) {
                DISTANCE += 1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            }

            if (DISTANCE < 0)
                DISTANCE = 0;

            if (gamepad1.right_stick_y >= 0.5) {
                kV -= 0.001;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            } else if (gamepad1.right_stick_y <= -0.5) {
                kV += 0.001;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            }

            if (gamepad1.left_bumper) {
                DriveConstantsPID.kV = kV;
                trajectory = drive.trajectoryBuilder()
                        .back(DISTANCE)
                        .build();
                drive = new SampleMecanumDriveREV(hardwareMap, false);
            }

            telemetry.addData("Instructions", "L stick to change distance. R stick to " +
                    "change kV. L bumper to save values.");

            telemetry.addData("DISTANCE", DISTANCE);
            telemetry.addData("kV", kV);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/StraightTestConstants_" +
                        System.currentTimeMillis() + ".txt", "Distance: " + DISTANCE + ", PID: " + coefficients + ", kV: "
                        + kV);
                break;
            }

            if (gamepad1.a && !blocker3) {
                drive.followTrajectorySync(trajectory);
                blocker3 = true;
            } else if (!gamepad1.a && blocker3) {
                blocker3 = false;
            }

            if (gamepad1.left_stick_y >= 0.5) {
                DISTANCE -= 1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            } else if (gamepad1.left_stick_y <= -0.5) {
                DISTANCE += 1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            }

            if (DISTANCE < 0)
                DISTANCE = 0;

            if (gamepad1.right_stick_y >= 0.5) {
                if (selected == 0) {
                    coefficients.kP -= 0.5;
                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                    }
                } else if (selected == 1) {
                    coefficients.kI -= 0.5;
                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                    }
                } else if (selected == 2) {
                    coefficients.kD -= 0.5;
                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                    }
                }
            } else if (gamepad1.right_stick_y <= -0.5) {
                if (selected == 0) {
                    coefficients.kP += 0.5;
                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                    }
                } else if (selected == 1) {
                    coefficients.kI += 0.5;
                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                    }
                } else if (selected == 2) {
                    coefficients.kD += 0.5;
                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                    }
                }
            }

            if (gamepad1.dpad_right && !blocker1) {
                if (selected < 2)
                    selected += 1;
                blocker1 = true;
            } else if (!gamepad1.dpad_right && blocker1) {
                blocker1 = false;
            }

            if (gamepad1.dpad_left && !blocker2) {
                if (selected > 0)
                    selected -= 1;
                blocker2 = true;
            } else if (!gamepad1.dpad_right && blocker2) {
                blocker2 = false;
            }

            if (gamepad1.left_bumper) {
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
                DriveConstantsPID.kV = kV;
                trajectory = drive.trajectoryBuilder()
                        .forward(DISTANCE)
                        .build();
            }

            telemetry.addData("Instructions", "A to begin moving. Press R bumper to save data. " + "DPAD L & R to select. L stick to change distance. R stick to " +
                    "change selected value. L bumper to save values.");

            if (selected == 0)
                telemetry.addData("SELECTED", "P");
            else if (selected == 1)
                telemetry.addData("SELECTED", "I");
            else if (selected == 2)
                telemetry.addData("SELECTED", "D");

            telemetry.addData("DISTANCE", DISTANCE);
            telemetry.addData("PID", coefficients);
            telemetry.addData("ERROR", drive.getLastError());
            telemetry.update();
        }

        drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, oldPID);
        DriveConstantsPID.kV = oldkV;
    }
}
