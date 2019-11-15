package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

/**
 *Created by Ethan
 */
public class HardwareOmnibot extends HardwareOmnibotDrive
{
    public enum AlignActivity {
        IDLE,
        ALIGN_TO_FOUNDATION,
        ALIGN_TO_WALL,
        REFINE_FOUNDATION,
        REFINE_WALL,
        STOPPING
    }

    public enum LiftActivity {
        IDLE,
		LOWERING_TO_GRAB,
        GRABBING_STONE,
        CLEARING_LIFT,
        LIFTING_TO_ROTATE,
        LIFTING_TO_STONE,
        ROTATING,
        LOWERING_TO_STONE
    }

    public enum ReleaseActivity {
        IDLE,
        LOWER_TO_RELEASE,
        RELEASE_STONE
    }

    public enum StowActivity {
        IDLE,
        RAISING_TO_ROTATE,
        ROTATING,
        CLEARING_LIFT,
        LOWERING_TO_STOW,
		OPENING_CLAW
    }

    public enum EjectActivity {
        IDLE,
        EJECT,
        RESET,
        STOPPING
    }

    public enum CapstoneActivity {
        IDLE,
        CLEARING_LIFT,
        LIFTING_TO_GRAB,
        ROTATING_TO_GRAB,
        LOWERING_TO_GRAB,
        GRABBING_CAPSTONE,
        LIFTING_CAPSTONE,
        ROTATING_TO_STONE,
        LOWERING_TO_STONE,
        RELEASING_CAPSTONE
    }

    public static int MAX_EXTENSION = 2375;
    public enum IntakePosition {
        RETRACTED(7),
		CAPSTONE(425),
		SPINMIN(1670),
		EJECT(2161),
		EXTENDED(MAX_EXTENSION);
        private final int encoderCount;

        IntakePosition(int encoderCount)
		{
			this.encoderCount = encoderCount;
		}

        public int getEncoderCount()
		{
			return encoderCount;
		}

		public static IntakePosition moveIn(IntakePosition currentPosition)
		{
			switch(currentPosition)
			{
				case RETRACTED:
				case CAPSTONE:
					return RETRACTED;
				case SPINMIN:
					return CAPSTONE;
				case EJECT:
					return SPINMIN;
				case EXTENDED:
					return EJECT;
				default:
					return currentPosition;
			}
		}

		public static IntakePosition moveOut(IntakePosition currentPosition)
		{
			switch(currentPosition)
			{
				case RETRACTED:
					return CAPSTONE;
				case CAPSTONE:
					return SPINMIN;
				case SPINMIN:
					return EJECT;
				case EJECT:
				case EXTENDED:
					return EXTENDED;
				default:
					return currentPosition;
			}
		}
	}

    public static int MAX_LIFT = 2800;
    public enum LiftPosition {
		GRABBING(0),
        STOWED(25),
        STONE1_RELEASE(126),
        STONE1(176),
        CAPSTONE_GRAB(300),
        CAPSTONE_ROTATE(400),
        STONE2_RELEASE(424),
        STONE2(474),
        STONE3_RELEASE(622),
        STONE3(672),
        ROTATE(850),
        STONE4_RELEASE(847),
        STONE4(907),
        STONE5_RELEASE(1071),
        STONE5(1131),
        STONE6_RELEASE(1297),
        STONE6(1357),
        STONE7_RELEASE(1533),
        STONE7(1593),
        STONE8_RELEASE(1750),
        STONE8(1810),
        STONE9_RELEASE(1970),
        STONE9(2030),
        STONE10_RELEASE(2191),
        STONE10(2251),
        STONE11_RELEASE(2415),
        STONE11(2475),
        STONE12_RELEASE(2620),
        STONE12(2680),
        LIFTMAX(MAX_LIFT);

        private final int encoderCount;

        LiftPosition(int encoderCount)
		{
			this.encoderCount = encoderCount;
		}

        public int getEncoderCount()
		{
			return encoderCount;
		}

		public static LiftPosition releasePosition(LiftPosition currentStone)
        {
            switch(currentStone)
            {
                case STONE1:
                    return STONE1_RELEASE;
                case STONE2:
                    return STONE2_RELEASE;
                case STONE3:
                    return STONE3_RELEASE;
                case STONE4:
                    return STONE4_RELEASE;
                case STONE5:
                    return STONE5_RELEASE;
                case STONE6:
                    return STONE6_RELEASE;
                case STONE7:
                    return STONE7_RELEASE;
                case STONE8:
                    return STONE8_RELEASE;
                case STONE9:
                    return STONE9_RELEASE;
                case STONE10:
                    return STONE10_RELEASE;
                case STONE11:
                    return STONE11_RELEASE;
                case STONE12:
                    return STONE12_RELEASE;
                default:
                    return currentStone;
            }
        }

		public static LiftPosition addStone(LiftPosition currentStone)
		{
			switch(currentStone)
			{
				case STONE1:
					return STONE2;
				case STONE2:
					return STONE3;
				case STONE3:
					return STONE4;
				case STONE4:
					return STONE5;
				case STONE5:
					return STONE6;
				case STONE6:
					return STONE7;
				case STONE7:
					return STONE8;
				case STONE8:
					return STONE9;
				case STONE9:
					return STONE10;
				case STONE10:
					return STONE11;
				case STONE11:
					return STONE12;
				default:
					return currentStone;
			}
		}

		public static LiftPosition removeStone(LiftPosition currentStone)
		{
			switch(currentStone)
			{
				case STONE2:
					return STONE1;
				case STONE3:
					return STONE2;
				case STONE4:
					return STONE3;
				case STONE5:
					return STONE4;
				case STONE6:
					return STONE5;
				case STONE7:
					return STONE6;
				case STONE8:
					return STONE7;
				case STONE9:
					return STONE8;
				case STONE10:
					return STONE9;
				case STONE11:
					return STONE10;
				case STONE12:
					return STONE11;
				default:
					return currentStone;
			}
		}
    }

    /* Public OpMode members. */
    public static double INTAKE_SPEED = 1.0;
	public static double LIFT_SPEED = 1.0;
	public static double LOWER_SPEED = 0.2;
	public static double EXTEND_SPEED = 1.0;
    public static double RIGHT_FINGER_DOWN = 0.32;
    public static double LEFT_FINGER_DOWN = 0.82;
    public static double RIGHT_FINGER_UP = 0.89;
    public static double LEFT_FINGER_UP = 0.25;
    public static double CLAW_OPEN = 0.4;
    public static double CLAW_PINCHED = 0.9;
    public static double CLAWDRICOPTER_FRONT = 0.85;
    public static double CLAWDRICOPTER_CAPSTONE = 0.67;
    public static double CLAWDRICOPTER_BACK = 0.09;
    public static int CLAW_OPEN_TIME = 500;
    public static int CLAW_CLOSE_TIME = 500;
    public static int CLAW_ROTATE_BACK_TIME = 1000;
    public static int CLAW_ROTATE_CAPSTONE_TIME = 500;
    public static int CLAW_ROTATE_FRONT_TIME = 1000;
	private static int ENCODER_ERROR = 10;

	// The OpMode set target height for the lift to go.
    public LiftPosition liftTargetHeight = LiftPosition.STONE1;
    // The height the activity was activated to achieve
	private LiftPosition liftActivityTargetHeight = LiftPosition.STONE1;
	public IntakePosition intakePosition = IntakePosition.RETRACTED;
	public IntakePosition intakeTargetPosition = IntakePosition.RETRACTED;

    // Robot Controller Config Strings
    public final static String RIGHT_FINGER = "RightFinger";
    public final static String LEFT_FINGER = "LeftFinger";
    public final static String CLAW = "Claw";
    public final static String CLAWDRICTOPTER = "Clawdricopter";
    public final static String LEFT_INTAKE = "LeftIntake";
    public final static String RIGHT_INTAKE = "RightIntake";
    public final static String LIFTER = "Lifter";
    public final static String EXTENDER = "Extender";
    public final static String RIGHT_RANGE = "RightTof";
    public final static String LEFT_RANGE = "LeftTof";
    public final static String BACK_RIGHT_RANGE = "BackRightTof";
    public final static String BACK_LEFT_RANGE = "BackLeftTof";
    public final static String BACK_RANGE = "BackTof";

    // Hardware objects
    protected Servo rightFinger = null;
    protected Servo leftFinger = null;
    protected Servo claw = null;
    protected Servo clawdricopter = null;
    protected DcMotor leftIntake = null;
    protected DcMotor rightIntake = null;
    protected DcMotor lifter = null;
    protected DcMotorEx extender = null;
    protected Rev2mDistanceSensor test = null;
    protected Rev2mTurbo rightTof = null;
    protected Rev2mTurbo leftTof = null;
    protected Rev2mTurbo backRightTof = null;
    protected Rev2mTurbo backLeftTof = null;
    protected Rev2mTurbo backTof = null;


    /* LEDs: Use this line if you drive the LEDs using an I2C/SPI bridge. */
    private DotStarBridgedLED leds;
    private IDotStarPattern robotDisplay;
    private IDotStarPattern ftcTimer;
    private IDotStarPattern halfAndHalf;
    private List<Integer> colors;

    // Tracking variables
    private ElapsedTime stateTimer;
    public LiftActivity liftState = LiftActivity.IDLE;
    public ReleaseActivity releaseState = ReleaseActivity.IDLE;
    public StowActivity stowState = StowActivity.IDLE;
    public EjectActivity ejectState = EjectActivity.IDLE;
    public AlignActivity alignState = AlignActivity.IDLE;
    public CapstoneActivity capstoneState = CapstoneActivity.IDLE;
    private boolean fingersUp = true;
    private boolean clawPinched = false;
    private boolean clawdricopterBack = false;
    // These are the heights of the stone levels to auto set the lift to
    protected LiftPosition lastLiftHeight = LiftPosition.STOWED;
    protected int liftZero = 0;
    protected int intakeZero = 0;
    public double intakePower = 0.0;
    protected boolean intakeZeroUpdated = false;

    protected double stackWallDistance = 0.0;
    protected double stackBackRightFoundationDistance = 0.0;
    protected double stackBackLeftFoundationDistance = 0.0;

	// Variables so we only read encoders once per loop
	protected boolean lifterEncoderRead = false;
	protected boolean extenderEncoderRead = false;
	protected boolean leftTofRead = false;
	protected boolean rightTofRead = false;
	protected boolean backRightTofRead = false;
	protected boolean backLeftTofRead = false;
    protected boolean backTofRead = false;
	protected int lifterEncoderValue = 0;
	protected int extenderEncoderValue = 0;
	protected double leftTofValue = 0.0;
	protected double rightTofValue = 0.0;
	protected double backRightTofValue = 0.0;
	protected double backLeftTofValue = 0.0;
	protected double backTofValue = 0.0;

    // Keeps the sensor from initializing more than once.
    public static boolean tofInitialized = false;
    // We can set this in Auto
    protected static boolean stackFromRightTof = false;

    /* Constructor */
    public HardwareOmnibot(){
        super();
    }

	public void resetReads() {
        super.resetReads();
		lifterEncoderRead = false;
		extenderEncoderRead = false;
		leftTofRead = false;
		rightTofRead = false;
		backRightTofRead = false;
		backLeftTofRead = false;
		backTofRead = false;
	}

    public void initGroundEffects()
    {
        // Use ModernRoboticsDIM if using Modern Robotics hardware.
        leds.setController(DotStarBridgedLED.Controller.RevExpansionHub);
        leds.setLength(60);
        colors = new ArrayList<>();
        colors.add(0, 0x0);
        colors.add(1, 0x0);
        halfAndHalf = new DSPatternHalfAndHalf(leds);
        halfAndHalf.setPatternColors(colors);
        ftcTimer = new DSPatternFtcTimer(leds);
        robotDisplay = halfAndHalf;
    }

    public void updateTimerGroundEffects() {
        robotDisplay.update();
    }

    public void startTimerGroundEffects() {
        robotDisplay = ftcTimer;
        robotDisplay.update();
    }

    public void stopGroundEffects() {
    }

    public void fingersDown() {
        rightFinger.setPosition(RIGHT_FINGER_DOWN);
        leftFinger.setPosition(LEFT_FINGER_DOWN);
    }

    public void fingersUp() {
        rightFinger.setPosition(RIGHT_FINGER_UP);
        leftFinger.setPosition(LEFT_FINGER_UP);
    }

    public boolean startCapstone() {
        boolean isCapping;
        if(capstoneState == CapstoneActivity.IDLE) {
            if((releaseState != ReleaseActivity.IDLE) || (stowState != StowActivity.IDLE) ||
                    (liftState != LiftActivity.IDLE)) {
                isCapping = false;
            } else {
                isCapping = true;
                // Extend the intake to make sure it isn't in the way.
                moveIntake(IntakePosition.EXTENDED);
                // We don't want the intake spinning while we are trying to lift the stone.
                // stopIntake();
                capstoneState = CapstoneActivity.CLEARING_LIFT;
            }
        } else {
            isCapping = true;
        }

        return isCapping;
    }

    public void performCapstone() {
        switch(capstoneState) {
            case RELEASING_CAPSTONE:
                if(stateTimer.milliseconds() >= CLAW_OPEN_TIME) {
                    capstoneState = CapstoneActivity.IDLE;
                }
                break;
            case LOWERING_TO_STONE:
                if(lifterAtPosition(LiftPosition.STOWED)) {
                    capstoneState = CapstoneActivity.RELEASING_CAPSTONE;
                    claw.setPosition(CLAW_OPEN);
                    clawPinched = false;
                    stateTimer.reset();
                }
                break;
            case ROTATING_TO_STONE:
                if(stateTimer.milliseconds() >= CLAW_ROTATE_CAPSTONE_TIME) {
                    capstoneState = CapstoneActivity.LOWERING_TO_STONE;
                    moveLift(LiftPosition.releasePosition(LiftPosition.STOWED));
                }
                break;
            case LIFTING_CAPSTONE:
                if(lifterAtPosition(LiftPosition.CAPSTONE_ROTATE)) {
                    capstoneState = CapstoneActivity.ROTATING_TO_STONE;
                    clawdricopter.setPosition(CLAWDRICOPTER_FRONT);
                    stateTimer.reset();
                }
                break;
            case GRABBING_CAPSTONE:
                if(stateTimer.milliseconds() >= CLAW_CLOSE_TIME) {
                    clawPinched = true;
                    capstoneState = CapstoneActivity.LIFTING_CAPSTONE;
                    moveLift(LiftPosition.releasePosition(LiftPosition.CAPSTONE_ROTATE));
                }
                break;
            case LOWERING_TO_GRAB:
                if(lifterAtPosition(LiftPosition.CAPSTONE_GRAB)) {
                    capstoneState = CapstoneActivity.GRABBING_CAPSTONE;
                    claw.setPosition(CLAW_PINCHED);
                    stateTimer.reset();
                }
                break;
            case ROTATING_TO_GRAB:
                if(stateTimer.milliseconds() >= CLAW_ROTATE_CAPSTONE_TIME) {
                    capstoneState = CapstoneActivity.LOWERING_TO_GRAB;
                    moveLift(LiftPosition.releasePosition(LiftPosition.CAPSTONE_GRAB));
                }
                break;
            case LIFTING_TO_GRAB:
                if(lifterAtPosition(LiftPosition.CAPSTONE_ROTATE)) {
                    capstoneState = CapstoneActivity.ROTATING_TO_GRAB;
                    clawdricopter.setPosition(CLAWDRICOPTER_CAPSTONE);
                    clawdricopterBack = false;
                    stateTimer.reset();
                }
                break;
            case CLEARING_LIFT:
                if(intakeAtPosition(IntakePosition.EXTENDED)) {
                    capstoneState = CapstoneActivity.LIFTING_TO_GRAB;
                    moveLift(LiftPosition.CAPSTONE_ROTATE);
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    public boolean startLifting() {
         boolean isLifting;
         if(liftState == LiftActivity.IDLE) {
            if((releaseState != ReleaseActivity.IDLE) || (stowState != StowActivity.IDLE)) {
                isLifting = false;
            } else {
                isLifting = true;
                // Extend the intake to make sure it isn't in the way.
                moveIntake(IntakePosition.EXTENDED);

                // We don't want the intake spinning while we are trying to lift the stone.
                stopIntake();
                liftState = LiftActivity.CLEARING_LIFT;
            }
        } else {
            isLifting = true;
        }

        return isLifting;
    }


    public void performLifting() {
		switch(liftState) {
            case LOWERING_TO_STONE:
                if(lifterAtPosition(liftActivityTargetHeight)) {
                    liftState = LiftActivity.IDLE;
                }
                break;
		    case ROTATING:
			    if(stateTimer.milliseconds() >= CLAW_ROTATE_BACK_TIME) {
                    if (liftActivityTargetHeight.getEncoderCount() <= LiftPosition.ROTATE.getEncoderCount()) {
                        liftState = LiftActivity.LOWERING_TO_STONE;
                        moveLift(liftActivityTargetHeight);
                    } else {
                        liftState = LiftActivity.IDLE;
                    }
					clawdricopterBack = true;
				}
			    break;
            case LIFTING_TO_STONE:
                if(lifterAtPosition(liftActivityTargetHeight)) {
                    liftState = LiftActivity.ROTATING;
                    clawdricopter.setPosition(CLAWDRICOPTER_BACK);
                    stateTimer.reset();
                }
                break;
		    case LIFTING_TO_ROTATE:
			    // It has gotten high enough
			    if(getLifterPosition() >= LiftPosition.ROTATE.getEncoderCount()) {
					liftState = LiftActivity.ROTATING;
					clawdricopter.setPosition(CLAWDRICOPTER_BACK);
					stateTimer.reset();
				}
			    break;
		    case GRABBING_STONE:
                if((stateTimer.milliseconds() >= CLAW_CLOSE_TIME) || clawPinched)
                {
					clawPinched = true;
					liftActivityTargetHeight = liftTargetHeight;
                    // If our target height is less than rotation height, we have to
                    // stop at rotation height first.
                    if(liftActivityTargetHeight.getEncoderCount() <= LiftPosition.ROTATE.getEncoderCount()) {
                        liftState = LiftActivity.LIFTING_TO_ROTATE;
                        moveLift(LiftPosition.ROTATE);
                    } else {
                        liftState = LiftActivity.LIFTING_TO_STONE;
                        moveLift(liftActivityTargetHeight);
                    }
                }
			    break;
		    case LOWERING_TO_GRAB:
			    // Has it gotten low enough
			    if(lifterAtPosition(LiftPosition.GRABBING)) {
					// Start grabbing the stone.  updateLifting will take over.
					liftState = LiftActivity.GRABBING_STONE;
					claw.setPosition(CLAW_PINCHED);
					stateTimer.reset();
				}
			    break;
            case CLEARING_LIFT:
			    if(intakeAtPosition(IntakePosition.EXTENDED)) {
                    liftState = LiftActivity.LOWERING_TO_GRAB;
					moveLift(LiftPosition.GRABBING);
				}
                break;
		    case IDLE:
			default:
				break;
		}
    }

    public boolean startReleasing() {
        boolean isReleasing;
        if(releaseState == ReleaseActivity.IDLE) {
            if (liftState != LiftActivity.IDLE) {
                isReleasing = false;
            } else if (stowState != StowActivity.IDLE) {
                isReleasing = false;
            } else {
                isReleasing = true;

                moveLift(LiftPosition.releasePosition(liftActivityTargetHeight));
                releaseState = ReleaseActivity.LOWER_TO_RELEASE;
            }
        }
        else {
            isReleasing = true;
        }

        return isReleasing;
    }

    public void performReleasing() {
        switch(releaseState)
        {
            case RELEASE_STONE:
                if((stateTimer.milliseconds() >= CLAW_OPEN_TIME) || !clawPinched)
                {
					clawPinched = false;
                    releaseState = ReleaseActivity.IDLE;
					// Add to our tower height for next lift.
					addStone();
					// Get the distance from the wall for alignment.
					if(stackFromRightTof)
                    {
                        stackWallDistance = readRightTof();
                    } else {
					    stackWallDistance = readLeftTof();
                    }
                    stackBackRightFoundationDistance = readBackRightTof();
                    stackBackLeftFoundationDistance = readBackLeftTof();
                }
                break;
            case LOWER_TO_RELEASE:
                if(lifterAtPosition(LiftPosition.releasePosition(liftActivityTargetHeight))) {
                    releaseState = ReleaseActivity.RELEASE_STONE;
                    claw.setPosition(CLAW_OPEN);
                    stateTimer.reset();
                }
                break;
            case IDLE:
		    default:
			    break;
        }
    }

    public boolean startStowing() {
        boolean isStowing;
        if(stowState == StowActivity.IDLE) {
            if (liftState != LiftActivity.IDLE) {
                isStowing = false;
            }
            if (releaseState != ReleaseActivity.IDLE) {
                isStowing = false;
            } else {
                isStowing = true;

                // Extend the intake to make sure it isn't in the way.
                moveIntake(IntakePosition.EXTENDED);

                // We don't want the intake spinning while we are trying to stow the
                // lift.
                stopIntake();
            }
            // Start rotating back to the front.
            stowState = StowActivity.CLEARING_LIFT;
        }
        else {
                isStowing = true;

        }
        return isStowing;
    }

    public void performStowing() {
		switch(stowState) {
			case LOWERING_TO_STOW:
			    if(lifterAtPosition(LiftPosition.STOWED)) {
					stowState = StowActivity.IDLE;
					startIntake(false);
				}
				break;
			case OPENING_CLAW:
			    if((stateTimer.milliseconds() >= CLAW_OPEN_TIME) || !clawPinched) {
                    stowState = StowActivity.LOWERING_TO_STOW;
                    clawPinched = false;
                }
				break;
			case ROTATING:
			    if((stateTimer.milliseconds() >= CLAW_ROTATE_FRONT_TIME) || !clawdricopterBack) {
					stowState = StowActivity.OPENING_CLAW;
					moveLift(LiftPosition.STOWED);
					clawdricopterBack = false;
				}
			    break;
			case RAISING_TO_ROTATE:
			    // It has gotten high enough
				if(clawdricopterBack) {
					if(lifterAtPosition(LiftPosition.ROTATE)) {
						stowState = StowActivity.ROTATING;
						clawdricopter.setPosition(CLAWDRICOPTER_FRONT);
						claw.setPosition(CLAW_OPEN);
						clawPinched = false;
						stateTimer.reset();
					}
				} else {
					stowState = StowActivity.ROTATING;
					claw.setPosition(CLAW_OPEN);
					clawPinched = false;
					stateTimer.reset();
				}
			    break;
            case CLEARING_LIFT:
			    if(intakeAtPosition(IntakePosition.EXTENDED)) {
                    stowState = StowActivity.RAISING_TO_ROTATE;
					if(clawdricopterBack) {
						moveLift(LiftPosition.ROTATE);
					}
                }
                break;
            case IDLE:
		    default:
			    break;
        }
    }

    // Maybe we will eventually get this virtual limit switch working.
    public boolean maxExtensionSetZero() {
        boolean maxExtended = false;
        // Have to make sure it is trying to extend fully.
        moveIntake(IntakePosition.EXTENDED);
        double extenderCurrentVelocity = extender.getVelocity();
        int intakeCurrentPosition = getIntakePosition();

        // The motor is stalled or we have finished
        if(extenderCurrentVelocity < 250) {
            int newZero = intakeCurrentPosition - MAX_EXTENSION;
            // We have reached target
            if(Math.abs(newZero) <= ENCODER_ERROR) {
                maxExtended = true;
            } else {
                // We missed our target
                maxExtended = false;
                setIntakeZero(newZero);
                moveIntake(IntakePosition.EXTENDED);
            }

        } else {
            // We are still running to position
            maxExtended = false;
        }

        return maxExtended;
    }

    public boolean distanceFromWall(double distance, double driveSpeed) {
        boolean targetReached = false;
        double wallDistance;
        double delta;
        double drivePower;
        if(stackFromRightTof) {
            wallDistance = readRightTof();
            delta = wallDistance - distance;
            if(Math.abs(delta) > 1.0) {
                // Need to drive left away from the wall.
                if(delta < 0) {
                    drivePower = -driveSpeed;
                // Need to drive right towards the wall.
                } else {
                    drivePower = driveSpeed;
                }
                drive(drivePower, 0.0, 0.0, -readIMU());
            } else {
                setAllDriveZero();
                targetReached = true;
            }
        } else {
            wallDistance = readLeftTof();
            delta = wallDistance - distance;
            if(Math.abs(delta) > 1.0) {
                // Need to drive right away from the wall.
                if(delta < 0) {
                    drivePower = driveSpeed;
                    // Need to drive left towards the wall.
                } else {
                    drivePower = -driveSpeed;
                }
                drive(drivePower, 0.0, 0.0, -readIMU());
            } else {
                setAllDriveZero();
                targetReached = true;
            }
        }

        return targetReached;
    }

    public boolean parallelRearTarget(double driveSpeed, double spinSpeed, double error) {
        boolean parallel = false;
        double leftDistance = readBackLeftTof();
        double rightDistance = readBackRightTof();
        double drivePower = 0.0;
        double spinPower = 0.0;
        double leftError = stackBackLeftFoundationDistance - leftDistance;
        double rightError = stackBackRightFoundationDistance - rightDistance;
        if((Math.abs(stackBackLeftFoundationDistance) > error) ||
                (Math.abs(rightError) > error)) {
            // Have to drive backwards towards the foundation
            if(((leftError) > error) &&
                    ((rightError) > error)) {
                drivePower = driveSpeed;
                // Have to spin ccw
                if(leftError > rightError) {
                    spinPower = -spinSpeed;
                // We have to spin cw
                } else if(rightError > leftError) {
                    spinPower = spinSpeed;
                }
            // Have to drive away from the foundation.
            } else if(((leftError) < -error) &&
                    ((rightError) < -error)) {
                drivePower = -driveSpeed;
                if(leftError > rightError) {
                    spinPower = -spinSpeed;
                    // We have to spin cw
                } else if(rightError > leftError) {
                    spinPower = spinSpeed;
                }
            // We just need to spin
            } else {
                drivePower = 0.0;
                if(leftError > rightError) {
                    spinPower = -spinSpeed;
                    // We have to spin cw
                } else if(rightError > leftError) {
                    spinPower = spinSpeed;
                }
            }
            drive(0, drivePower, spinPower, -readIMU());
        } else {
            setAllDriveZero();
            parallel = true;
        }

        return parallel;
    }

    public void stopAligning() {
        alignState = AlignActivity.STOPPING;
    }

    public boolean startAligning() {
        boolean aligning = false;
        if (alignState == AlignActivity.IDLE) {
            aligning = true;
            alignState = AlignActivity.ALIGN_TO_FOUNDATION;
            parallelRearTarget(0.07, 0.07, 1.0);
        }

        return aligning;
    }

    public void performAligning() {
        switch(alignState)
        {
            case REFINE_WALL:
                if(distanceFromWall(stackWallDistance,0.05)) {
                    alignState = AlignActivity.REFINE_FOUNDATION;
                }
                break;
            case REFINE_FOUNDATION:
                if(parallelRearTarget(0.05, 0.05, 0.5)) {
                    alignState = AlignActivity.IDLE;
                }
                break;
            case ALIGN_TO_WALL:
                if(distanceFromWall(stackWallDistance,0.07)) {
                    alignState = AlignActivity.REFINE_WALL;
                }
                break;
            case ALIGN_TO_FOUNDATION:
                if(parallelRearTarget(0.07, 0.07, 1.0)) {
                    alignState = AlignActivity.ALIGN_TO_WALL;
                }
                break;
            case STOPPING:
                setAllDriveZero();
                alignState = AlignActivity.IDLE;
            case IDLE:
            default:
                break;
        }
    }

    public void startEjecting() {
        if (ejectState == EjectActivity.IDLE) {
            ejectState = EjectActivity.EJECT;
            startIntake(true);
            moveIntake(IntakePosition.EJECT);
        }
    }

    public void performEjecting() {
        switch(ejectState)
        {
            case EJECT:
                if(intakeAtPosition(IntakePosition.EJECT)) {
                    ejectState = EjectActivity.RESET;
                    startIntake(false);
                    moveIntake(IntakePosition.EXTENDED);
                }
                break;
            case RESET:
                if(intakeAtPosition(IntakePosition.EXTENDED)) {
                    ejectState = EjectActivity.IDLE;
                }
                break;
            case STOPPING:
                ejectState = EjectActivity.IDLE;
            case IDLE:
            default:
                break;
        }
    }

    public void addStone() {
        liftTargetHeight = LiftPosition.addStone(liftTargetHeight);
    }

    public void removeStone() {
        liftTargetHeight = LiftPosition.removeStone(liftTargetHeight);
    }

    public void intakeOut() {
        intakeTargetPosition = IntakePosition.moveOut(intakeTargetPosition);
    }

    public void intakeIn() {
        intakeTargetPosition = IntakePosition.moveIn(intakeTargetPosition);
    }

    public void moveIntake(IntakePosition targetIntakePosition) {
		if((targetIntakePosition != intakePosition) || (intakeZeroUpdated)) {
		    int targetPosition = targetIntakePosition.getEncoderCount();
		    // Make sure the intake isn't spinning if we retract too far.
		    if(targetPosition < IntakePosition.SPINMIN.getEncoderCount()) {
		        stopIntake();
            }
            setIntakePosition(targetPosition);
            extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extender.setPower(EXTEND_SPEED);
			intakePosition = targetIntakePosition;
		}
    }

    public void moveLift(LiftPosition targetHeight) {
        if(getIntakePosition() >= IntakePosition.CAPSTONE.getEncoderCount()) {
            int liftPosition = getLifterPosition();
            int targetPosition = targetHeight.getEncoderCount();
            double lifterSpeed;
            if (liftPosition < targetPosition) {
                lifterSpeed = LIFT_SPEED;
            } else {
                lifterSpeed = LOWER_SPEED;
            }
			setLifterPosition(targetPosition);
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setPower(lifterSpeed);
        }
    }

    public void startIntake(boolean reverse) {
        // Prevent the intake from starting if it isn't extended far enough.
        if(getIntakePosition() >= IntakePosition.SPINMIN.getEncoderCount()) {
            if (reverse) {
                if(intakePower != -INTAKE_SPEED) {
                    stopIntake();
                    intakePower = -INTAKE_SPEED;
                    leftIntake.setPower(intakePower);
                    rightIntake.setPower(intakePower);
                }
            } else {
                if(intakePower != INTAKE_SPEED) {
                    stopIntake();
                    intakePower = INTAKE_SPEED;
                    leftIntake.setPower(intakePower);
                    rightIntake.setPower(intakePower);
                }
            }
        }
    }

    public void stopIntake() {
        if(intakePower != 0.0) {
            intakePower = 0.0;
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);
        }
    }

	public boolean lifterAtPosition(LiftPosition targetPosition) {
		return Math.abs(getLifterPosition() - targetPosition.getEncoderCount()) < ENCODER_ERROR;
	}

    public void setLiftZero(int value) {
        liftZero = value;
    }

    public int getLifterPosition() {
		if(!lifterEncoderRead) {
			lifterEncoderValue = lifter.getCurrentPosition();
			lifterEncoderRead = true;
		}

        return lifterEncoderValue - liftZero;
    }

    public void setLifterPosition(int targetPosition) {
        lifter.setTargetPosition(targetPosition + liftZero);
    }

	public boolean intakeAtPosition(IntakePosition targetPosition) {
		return Math.abs(getIntakePosition() - targetPosition.getEncoderCount()) < ENCODER_ERROR;
	}

    public void setIntakeZero(int value) {
        intakeZeroUpdated = true;
        intakeZero = value;
    }

    public int getIntakePosition() {
		if(!extenderEncoderRead) {
			extenderEncoderValue = extender.getCurrentPosition();
			extenderEncoderRead = true;
		}
        return extenderEncoderValue - intakeZero;
    }

    public void setIntakePosition(int targetPosition) {
		extender.setTargetPosition(targetPosition + intakeZero);
    }

    public double readLeftTof() {
        if(!leftTofRead) {
            leftTofRead = true;
            leftTofValue = leftTof.getDistance(DistanceUnit.CM);
        }

        return leftTofValue;
    }

    public double readRightTof() {
        if(!rightTofRead) {
            rightTofRead = true;
            rightTofValue = rightTof.getDistance(DistanceUnit.CM);
        }

        return rightTofValue;
    }

    public double readBackLeftTof() {
        if(!backLeftTofRead) {
            backLeftTofRead = true;
            backLeftTofValue = backLeftTof.getDistance(DistanceUnit.CM);
        }

        return backLeftTofValue;
    }

    public double readBackRightTof() {
        if(!backRightTofRead) {
            backRightTofRead = true;
            backRightTofValue = backRightTof.getDistance(DistanceUnit.CM);
        }

        return backRightTofValue;
    }

    public double readBackTof() {
        if(!backTofRead) {
            backTofRead = true;
            backTofValue = backTof.getDistance(DistanceUnit.CM);
        }

        return backTofValue;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        super.init(ahwMap);

        stateTimer = new ElapsedTime();
        rightFinger = hwMap.get(Servo.class, RIGHT_FINGER);
        leftFinger = hwMap.get(Servo.class, LEFT_FINGER);
        claw = hwMap.get(Servo.class, CLAW);
        clawdricopter = hwMap.get(Servo.class, CLAWDRICTOPTER);
        leftIntake = hwMap.get(DcMotor.class, LEFT_INTAKE);
        rightIntake = hwMap.get(DcMotor.class, RIGHT_INTAKE);
        lifter = hwMap.get(DcMotor.class, LIFTER);
        extender = hwMap.get(DcMotorEx.class, EXTENDER);

        rightTof = (Rev2mTurbo)hwMap.get(DistanceSensor.class, RIGHT_RANGE);
        leftTof = (Rev2mTurbo)hwMap.get(DistanceSensor.class, LEFT_RANGE);
        backRightTof = (Rev2mTurbo)hwMap.get(DistanceSensor.class, BACK_RIGHT_RANGE);
        backLeftTof = (Rev2mTurbo)hwMap.get(DistanceSensor.class, BACK_LEFT_RANGE);
        backTof = (Rev2mTurbo)hwMap.get(DistanceSensor.class, BACK_RANGE);
        if(!tofInitialized) {
            rightTof.initVL53L0X(false);
            leftTof.initVL53L0X(false);
            backRightTof.initVL53L0X(false);
            backLeftTof.initVL53L0X(false);
            backTof.initVL53L0X(false);
            tofInitialized = true;
        }

        // Set motor rotation
        // This makes lift go up with positive encoder values and power
        lifter.setDirection(DcMotor.Direction.REVERSE);
        // This makes extender go out with positive encoder values and power
        extender.setDirection(DcMotor.Direction.FORWARD);
        // This makes intake pull in with positive encoder values and power
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        // Set motor encoder usage
        // Lifter encoder allows us to go to specific heights.
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Extender encoder allows us to extend to specific positions.
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The intake should just run as fast as it can.
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset the encoders that are used.
        int sleepTime = 0;
        int encoderCount = lifter.getCurrentPosition();

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            encoderCount = lifter.getCurrentPosition();
        }

        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set the stop mode
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fingersUp();

        // Set up the LEDs. Change this to your configured name.
        leds = hwMap.get(DotStarBridgedLED.class, "leds");

        initGroundEffects();
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    /*
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    */
}

