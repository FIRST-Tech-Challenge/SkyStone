package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by 12090 STEM Punk
 */
public class HardwareOmnibot extends HardwareOmnibotDrive
{
    public enum StackActivities {
        IDLE,
        LIFT,
        RELEASE,
        STOW
    }

    public enum FoundationActivities {
        IDLE,
        GRAB,
        SETTLE,
        ACCEL,
        DECEL,
        STOP
    }

    public enum RobotSide {
        FRONT,
        BACK,
        RIGHT,
        LEFT
    }

    public enum ControlledAcceleration {
        IDLE,
        ACCELERATING,
        STOPPING
    }

    public enum ControlledDeceleration {
        IDLE,
        DECELERATING,
        STOPPING
    }

    public enum GrabFoundationActivity {
        IDLE,
        BACKING_UP,
        GRABBING,
        STOPPING
    }

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
        LOWER_TO_RELEASE
//        RELEASE_STONE
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
        STOWED(15),
        STONE1_RELEASE(126),
        STONE1(176),
        STONE1_ROTATE(226),
        CAPSTONE_GRAB(300),
        STONE2_RELEASE(424),
        STONE2(474),
        STONE2_ROTATE(424),
        CAPSTONE_ROTATE(650),
        STONE3_RELEASE(622),
        STONE3(672),
        STONE3_ROTATE(722),
        ROTATE(850),
        STONE4_RELEASE(847),
        STONE4(907),
        STONE4_ROTATE(957),
        STONE5_RELEASE(1071),
        STONE5(1131),
        STONE5_ROTATE(1181),
        STONE6_RELEASE(1297),
        STONE6(1357),
        STONE6_ROTATE(1407),
        STONE7_RELEASE(1533),
        STONE7(1593),
        STONE7_ROTATE(1643),
        STONE8_RELEASE(1750),
        STONE8(1810),
        STONE8_ROTATE(1860),
        STONE9_RELEASE(1970),
        STONE9(2030),
        STONE9_ROTATE(2080),
        STONE10_RELEASE(2191),
        STONE10(2251),
        STONE10_ROTATE(2301),
        STONE11_RELEASE(2415),
        STONE11(2475),
        STONE11_ROTATE(2525),
        STONE12_RELEASE(2620),
        STONE12(2680),
        STONE12_ROTATE(2730),
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

        public static LiftPosition rotatePosition(LiftPosition currentStone)
        {
            switch(currentStone)
            {
                case ROTATE:
                    return ROTATE;
                case STONE1:
                    return STONE1_ROTATE;
                case STONE2:
                    return STONE2_ROTATE;
                case STONE3:
                    return STONE3_ROTATE;
                case STONE4:
                    return STONE4_ROTATE;
                case STONE5:
                    return STONE5_ROTATE;
                case STONE6:
                    return STONE6_ROTATE;
                case STONE7:
                    return STONE7_ROTATE;
                case STONE8:
                    return STONE8_ROTATE;
                case STONE9:
                    return STONE9_ROTATE;
                case STONE10:
                    return STONE10_ROTATE;
                case STONE11:
                    return STONE11_ROTATE;
                case STONE12:
                    return STONE12_ROTATE;
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
    public final static double ACCEL_STEP = 0.007;
    public static double INTAKE_SPEED = 1.0;
	public static double LIFT_MAX_SPEED = 1.0;
	public static double LIFT_MID_SPEED = 0.5;
	public static double LIFT_MIN_SPEED = 0.2;
	public static double EXTEND_SPEED = 1.0;
    public static double RIGHT_FINGER_DOWN = 0.32;
    public static double LEFT_FINGER_DOWN = 0.82;
    public static double RIGHT_FINGER_UP = 0.89;
    public static double LEFT_FINGER_UP = 0.25;
    public static double CLAW_OPEN = 0.4;
    public static double CLAW_PINCHED = 0.9;
    public static double CLAWDRICOPTER_FRONT = 0.85;
    public static double CLAWDRICOPTER_CAPSTONE = 0.70;
    public static double CLAWDRICOPTER_BACK = 0.09;
    public static int CLAW_OPEN_TIME = 500;
    public static int CLAW_CLOSE_TIME = 500;
    public static int CLAW_ROTATE_BACK_TIME = 1000;
    public static int CLAW_ROTATE_CAPSTONE_TIME = 500;
    public static int CLAW_ROTATE_FRONT_TIME = 1000;
    public static int FINGER_ROTATE_TIME = 500;
	private static int ENCODER_ERROR = 10;
    RevBulkData bulkDataHub1;
    RevBulkData bulkDataHub2;
//    ExpansionHubMotor leftIntakeBd, rightIntakeBd, lifterBd, extenderBd;
    boolean hub1Read = false;
    ExpansionHubEx expansionHub1;
    boolean hub2Read = false;
    ExpansionHubEx expansionHub2;

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
    public final static String HUB1 = "Expansion Hub 2";
    public final static String HUB2 = "Expansion Hub 3";

    // Hardware objects
    protected Servo rightFinger = null;
    protected Servo leftFinger = null;
    protected Servo claw = null;
    protected Servo clawdricopter = null;
    protected ExpansionHubMotor leftIntake = null;
    protected ExpansionHubMotor rightIntake = null;
    protected ExpansionHubMotor lifter = null;
    protected ExpansionHubMotor extender = null;
    protected Rev2mDistanceSensor rightTof = null;
    protected Rev2mDistanceSensor leftTof = null;
    protected Rev2mDistanceSensor backRightTof = null;
    protected Rev2mDistanceSensor backLeftTof = null;
    protected Rev2mDistanceSensor backTof = null;


    /* LEDs: Use this line if you drive the LEDs using an I2C/SPI bridge. */
    private DotStarBridgedLED leds;
    private IDotStarPattern robotDisplay;
    private IDotStarPattern ftcTimer;
    private IDotStarPattern halfAndHalf;
    private List<Integer> colors;

    // Tracking variables
    private ElapsedTime clawTimer;
    private ElapsedTime fingerTimer;
    private ElapsedTime settleTimer;
    private ElapsedTime clawdricopterTimer;
    public LiftActivity liftState = LiftActivity.IDLE;
    public ReleaseActivity releaseState = ReleaseActivity.IDLE;
    public StowActivity stowState = StowActivity.IDLE;
    public EjectActivity ejectState = EjectActivity.IDLE;
    public AlignActivity alignState = AlignActivity.IDLE;
    public GrabFoundationActivity grabState = GrabFoundationActivity.IDLE;
    public ControlledAcceleration accelerationState = ControlledAcceleration.IDLE;
    public ControlledDeceleration decelerationState = ControlledDeceleration.IDLE;
    public CapstoneActivity capstoneState = CapstoneActivity.IDLE;
    public FoundationActivities removeFoundation = FoundationActivities.IDLE;
    public StackActivities stackStone = StackActivities.IDLE;
    private boolean clawPinched = false;
    private boolean clawdricopterBack = false;
    // These are the heights of the stone levels to auto set the lift to
    protected LiftPosition lastLiftHeight = LiftPosition.STOWED;
    protected static int liftZero = 0;
    protected static int intakeZero = 0;
    public double intakePower = 0.0;
    protected boolean intakeZeroUpdated = false;
    protected boolean stowingLift = false;

    protected double stackWallDistance = 0.0;
    protected double stackBackRightFoundationDistance = 0.0;
    protected double stackBackLeftFoundationDistance = 0.0;

	// Variables so we only read encoders once per loop
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
	protected double accelerationFinal = 0.0;
	protected double accelerationCurrent = 0.0;
	protected RobotSide accelerationSide = RobotSide.FRONT;
    protected double decelerationStart = 0.0;
    protected double decelerationCurrent = 0.0;
    protected RobotSide decelerationSide = RobotSide.FRONT;

    // Keeps the sensor from initializing more than once.
    public static boolean tofInitialized = false;
    // We can set this in Auto
    protected static RobotSide stackFromSide = RobotSide.RIGHT;

    /* Constructor */
    public HardwareOmnibot(){
        super();
    }

	public void resetReads() {
        super.resetReads();
        // Bulk data items
        hub1Read = false;
        hub2Read = false;

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
                if(clawTimer.milliseconds() >= CLAW_OPEN_TIME) {
                    capstoneState = CapstoneActivity.IDLE;
                }
                break;
            case LOWERING_TO_STONE:
                if(lifterAtPosition(LiftPosition.STOWED)) {
                    capstoneState = CapstoneActivity.RELEASING_CAPSTONE;
                    claw.setPosition(CLAW_OPEN);
                    clawPinched = false;
                    clawTimer.reset();
                }
                break;
            case ROTATING_TO_STONE:
                if(clawdricopterTimer.milliseconds() >= CLAW_ROTATE_CAPSTONE_TIME) {
                    capstoneState = CapstoneActivity.LOWERING_TO_STONE;
                    moveLift(LiftPosition.releasePosition(LiftPosition.STOWED));
                }
                break;
            case LIFTING_CAPSTONE:
                if(lifterAtPosition(LiftPosition.CAPSTONE_ROTATE)) {
                    capstoneState = CapstoneActivity.ROTATING_TO_STONE;
                    clawdricopter.setPosition(CLAWDRICOPTER_FRONT);
                    clawdricopterTimer.reset();
                }
                break;
            case GRABBING_CAPSTONE:
                if(clawTimer.milliseconds() >= CLAW_CLOSE_TIME) {
                    clawPinched = true;
                    capstoneState = CapstoneActivity.LIFTING_CAPSTONE;
                    moveLift(LiftPosition.releasePosition(LiftPosition.CAPSTONE_ROTATE));
                }
                break;
            case LOWERING_TO_GRAB:
                if(lifterAtPosition(LiftPosition.CAPSTONE_GRAB)) {
                    capstoneState = CapstoneActivity.GRABBING_CAPSTONE;
                    claw.setPosition(CLAW_PINCHED);
                    clawTimer.reset();
                }
                break;
            case ROTATING_TO_GRAB:
                if(clawdricopterTimer.milliseconds() >= CLAW_ROTATE_CAPSTONE_TIME) {
                    capstoneState = CapstoneActivity.LOWERING_TO_GRAB;
                    moveLift(LiftPosition.releasePosition(LiftPosition.CAPSTONE_GRAB));
                }
                break;
            case LIFTING_TO_GRAB:
                if(lifterAtPosition(LiftPosition.CAPSTONE_ROTATE)) {
                    capstoneState = CapstoneActivity.ROTATING_TO_GRAB;
                    clawdricopter.setPosition(CLAWDRICOPTER_CAPSTONE);
                    clawdricopterBack = false;
                    clawdricopterTimer.reset();
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
			    if(clawdricopterTimer.milliseconds() >= CLAW_ROTATE_BACK_TIME) {
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
                    clawdricopterTimer.reset();
                }
                break;
		    case LIFTING_TO_ROTATE:
			    // It has gotten high enough
			    if(getLifterPosition() >= LiftPosition.ROTATE.getEncoderCount()) {
					liftState = LiftActivity.ROTATING;
					clawdricopter.setPosition(CLAWDRICOPTER_BACK);
					clawdricopterTimer.reset();
				}
			    break;
		    case GRABBING_STONE:
                if((clawTimer.milliseconds() >= CLAW_CLOSE_TIME) || clawPinched)
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
					clawTimer.reset();
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
//            case RELEASE_STONE:
//                if((clawTimer.milliseconds() >= CLAW_OPEN_TIME) || !clawPinched)
//                {
//					clawPinched = false;
//                    releaseState = ReleaseActivity.IDLE;
//					// Add to our tower height for next lift.
//					addStone();
//					// Get the distance from the wall for alignment.
//					if(stackFromSide == AlignmentSide.RIGHT)
//                    {
//                        stackWallDistance = readRightTof();
//                    } else {
//					    stackWallDistance = readLeftTof();
//                    }
//                    stackBackRightFoundationDistance = readBackRightTof();
//                    stackBackLeftFoundationDistance = readBackLeftTof();
//                }
//                break;
            case LOWER_TO_RELEASE:
                if(lifterAtPosition(LiftPosition.releasePosition(liftActivityTargetHeight))) {
                    releaseState = ReleaseActivity.IDLE;
//                    claw.setPosition(CLAW_OPEN);
//                    clawTimer.reset();
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
            } else if (releaseState != ReleaseActivity.IDLE) {
                isStowing = false;
            } else {
                isStowing = true;
                stowingLift = true;

                // Extend the intake to make sure it isn't in the way.
                moveIntake(IntakePosition.EXTENDED);

                // We don't want the intake spinning while we are trying to stow the
                // lift.
                stopIntake();

                // Open the claw to release stone.
                claw.setPosition(CLAW_OPEN);
                clawTimer.reset();

                // Start rotating back to the front.
                stowState = StowActivity.CLEARING_LIFT;
            }
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
                    stowingLift = false;
				}
				break;
			case ROTATING:
			    if((clawdricopterTimer.milliseconds() >= CLAW_ROTATE_FRONT_TIME) || !clawdricopterBack) {
					stowState = StowActivity.LOWERING_TO_STOW;
					moveLift(LiftPosition.STOWED);
					clawdricopterBack = false;
				}
			    break;
			case RAISING_TO_ROTATE:
			    // It has gotten high enough
				if(clawdricopterBack) {
					if(lifterAtPosition(LiftPosition.releasePosition(liftActivityTargetHeight))) {
						stowState = StowActivity.ROTATING;
						clawdricopter.setPosition(CLAWDRICOPTER_FRONT);
						clawdricopterTimer.reset();
					}
				} else {
					stowState = StowActivity.ROTATING;
				}
			    break;
            case OPENING_CLAW:
                if((clawTimer.milliseconds() >= CLAW_OPEN_TIME) || !clawPinched)
                {
                    // This happens if we are stowing from a non-stone position
                    // like starting the match.
                    if(clawPinched) {
                        addStone();
                        clawPinched = false;
                    }
                    stowState = StowActivity.RAISING_TO_ROTATE;
					// Add to our tower height for next lift.
					// Get the distance from the wall for alignment.
					if(stackFromSide == RobotSide.RIGHT)
                    {
                        stackWallDistance = readRightTof();
                    } else {
					    stackWallDistance = readLeftTof();
                    }
                    stackBackRightFoundationDistance = readBackRightTof();
                    stackBackLeftFoundationDistance = readBackLeftTof();
                    stowState = StowActivity.RAISING_TO_ROTATE;
                    if(clawdricopterBack) {
                        if (liftActivityTargetHeight.getEncoderCount() < LiftPosition.ROTATE.getEncoderCount()) {
                            liftActivityTargetHeight = LiftPosition.ROTATE;
                        }
                        moveLift(LiftPosition.releasePosition(liftActivityTargetHeight));
                    }
                }
                break;
            case CLEARING_LIFT:
			    if(intakeAtPosition(IntakePosition.EXTENDED)) {
                    stowState = StowActivity.OPENING_CLAW;
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

    public boolean distanceFromWall(RobotSide side, double distance, double driveSpeed, double error) {
        boolean targetReached = false;
        double wallDistance = 0;
        double delta;
        double drivePower;
		double driveAngle = 0;
		double maxRange = 40.0;
		double maxMultiplier = 0.4;
		double midRange = 20.0;
		double midMultiplier = 0.2;
		double minRange = 10.0;
		double minMultiplier = 0.05;
        switch(side)
        {
            case FRONT:
                // Currently no sensor on the front.
                break;
            case BACK:
                wallDistance = readBackTof();
				driveAngle = 90;
                break;
            case LEFT:
                wallDistance = readLeftTof();
				driveAngle = 180;
                break;
            case RIGHT:
                wallDistance = readRightTof();
				driveAngle = 0;
                break;
        }
        delta = wallDistance - distance;
		// Slow down as distance is approached
        if(Math.abs(delta) > error) {
			if(delta < 0) {
				// Need to drive forward away from the wall.
				drivePower = -driveSpeed;
			} else {
				// Need to drive backward towards the wall.
				drivePower = driveSpeed;
			}

			// Go at slowest speed.
			if(Math.abs(delta) <= minRange) {
				drivePower *= minMultiplier;
			} else if(Math.abs(delta) <= midRange) {
				drivePower *= midMultiplier;
			} else if (Math.abs(delta) < maxRange) {
				drivePower *= maxMultiplier;
			}
			// make sure we don't go below minimum drive power.
			if(Math.abs(drivePower) < MIN_DRIVE_RATE) {
				drivePower = Math.copySign(MIN_DRIVE_RATE, drivePower);
			}
            drive(drivePower, 0.0, 0.0, driveAngle-readIMU(), false);
		} else {
            setAllDriveZero();
            targetReached = true;
		}

        return targetReached;
    }

    // This function goes to a distance of two range sensors to maintain angle from object
    public boolean parallelRearTarget(double backLeftDistance, double backRightDistance, double driveSpeed, double spinSpeed, double error) {
        boolean parallel = false;
        double leftDistance = readBackLeftTof();
        double rightDistance = readBackRightTof();
        double drivePower = 0.0;
        double spinPower = 0.0;
        double leftError = backLeftDistance - leftDistance;
        double rightError = backRightDistance - rightDistance;
        if((Math.abs(leftError) > error) || (Math.abs(rightError) > error)) {
            // Have to drive backwards towards the foundation
            if(((leftError) > error) &&
                    ((rightError) > error)) {
                drivePower = driveSpeed;
                // Have to spin ccw
                if(leftError > rightError) {
                    spinPower = spinSpeed;
                // We have to spin cw
                } else if(rightError > leftError) {
                    spinPower = -spinSpeed;
                }
            // Have to drive away from the foundation.
            } else if(((leftError) < -error) &&
                    ((rightError) < -error)) {
                drivePower = -driveSpeed;
                if(leftError > rightError) {
                    spinPower = spinSpeed;
                    // We have to spin cw
                } else if(rightError > leftError) {
                    spinPower = -spinSpeed;
                }
            // We just need to spin
            } else {
                drivePower = 0.0;
                if(leftError > rightError) {
                    spinPower = spinSpeed;
                    // We have to spin cw
                } else if(rightError > leftError) {
                    spinPower = -spinSpeed;
                }
            }
            drive(0, drivePower, spinPower, -readIMU(), false);
        } else {
            setAllDriveZero();
            parallel = true;
        }

        return parallel;
    }

    // This function backs up to an object
    public boolean gotoRearTarget(double driveSpeed, double spinSpeed) {
        // These are the values we see when it is against the platform.
        double backLeftTargetDistance = 1.0;
        double backRightTargetDistance = 3.5;
        double leftTofDistance = readBackLeftTof();
        double rightTofDistance = readBackRightTof();
        double drivePower = 0.0;
        double spinPower = 0.0;
        double leftError = backLeftTargetDistance - leftTofDistance;
        double rightError = backRightTargetDistance - rightTofDistance;
        boolean touching = false;

        // Slow down for 10cm
        double slowDownStart = 20.0;
        double slowDownRange = 10.0;
        if((leftError < 0) || (rightError < 0)) {
            // Have to drive backwards towards the foundation
            drivePower = driveSpeed;

            // If one of them is within range, drive speed should be minimum, all
            // rotation.
            if(!((rightError < 0) && (leftError < 0))) {
                drivePower = MIN_DRIVE_RATE;
                spinPower = Math.copySign(spinPower, MIN_SPIN_RATE);
            } else {
                // We want the one closer to the foundation.  minError should be negative.
                double minError = Math.max(rightError, leftError);

                // Need to set drive power based on range.
                // Go at slowest speed.
                double scaleFactor = 0.95 * (slowDownRange - (slowDownStart + minError))/slowDownRange + MIN_DRIVE_RATE;
                scaleFactor = Math.min(1.0, scaleFactor);
                drivePower = driveSpeed * scaleFactor;
            }
            // make sure we don't go below minimum spin power.
            if(spinPower < MIN_SPIN_RATE) {
                spinPower = Math.copySign(spinPower, MIN_SPIN_RATE);
            }
            // make sure we don't go below minimum drive power.
            if(drivePower < MIN_DRIVE_RATE) {
                drivePower = MIN_DRIVE_RATE;
            }
            // Scale the power based on how far
            drive(0, -drivePower, -spinPower, -readIMU(), false);
        } else {
            drive(0, -MIN_DRIVE_RATE, 0, -readIMU(), false);
            touching = true;
        }

        return touching;
    }

    public void stopAligning() {
        alignState = AlignActivity.STOPPING;
    }

    public boolean startAligning() {
        boolean aligning = false;
        if (alignState == AlignActivity.IDLE) {
            aligning = true;
            alignState = AlignActivity.ALIGN_TO_FOUNDATION;
            parallelRearTarget(stackBackLeftFoundationDistance, stackBackRightFoundationDistance, 0.07, 0.07, 1.0);
        }

        return aligning;
    }

    public void performAligning() {
        switch(alignState)
        {
            case REFINE_WALL:
                if(distanceFromWall(stackFromSide, stackWallDistance,0.05, 1.0)) {
                    alignState = AlignActivity.REFINE_FOUNDATION;
                }
                break;
            case REFINE_FOUNDATION:
                if(parallelRearTarget(stackBackLeftFoundationDistance, stackBackRightFoundationDistance, 0.05, 0.05, 0.5)) {
                    alignState = AlignActivity.IDLE;
                }
                break;
            case ALIGN_TO_WALL:
                if(distanceFromWall(stackFromSide, stackWallDistance,0.07, 1.0)) {
                    alignState = AlignActivity.REFINE_WALL;
                }
                break;
            case ALIGN_TO_FOUNDATION:
                if(parallelRearTarget(stackBackLeftFoundationDistance, stackBackRightFoundationDistance, 0.07, 0.07, 1.0)) {
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

    public boolean startAccelerating(double finalSpeed, RobotSide driveDirection) {
        boolean isAccelerating;
        if (accelerationState == ControlledAcceleration.IDLE) {
            isAccelerating = true;
            accelerationState = ControlledAcceleration.ACCELERATING;
            accelerationFinal = finalSpeed;
            accelerationSide = driveDirection;
            accelerationCurrent = 0.0;
        } else {
            isAccelerating = true;
        }

        return isAccelerating;
    }

    public void stopAccelerating() {
        if (accelerationState != ControlledAcceleration.IDLE) {
            accelerationState = ControlledAcceleration.IDLE;
            setAllDriveZero();
        }
    }

    public void performAcceleration() {
        switch(accelerationState) {
            case ACCELERATING:
                accelerationCurrent += ACCEL_STEP;
                if(accelerationCurrent > accelerationFinal) {
                    accelerationCurrent = accelerationFinal;
                    accelerationState = ControlledAcceleration.IDLE;
                    // Just for testing, do the accel and decel at once.
                    startDecelerating(accelerationCurrent, accelerationSide);
                }
                switch(accelerationSide) {
                    case LEFT:
                        drive(-accelerationCurrent, 0, 0, -readIMU(), false);
                        break;
                    case RIGHT:
                        drive(accelerationCurrent, 0, 0, -readIMU(), false);
                        break;
                    case BACK:
                        drive(0, -accelerationCurrent, 0, -readIMU(), false);
                        break;
                    case FRONT:
                        drive(0, accelerationCurrent, 0, -readIMU(), false);
                        break;
                }
                break;
            case STOPPING:
                setAllDriveZero();
                accelerationState = ControlledAcceleration.IDLE;
            case IDLE:
                break;
        }
    }

    public boolean startDecelerating(double startSpeed, RobotSide driveDirection) {
        boolean isDecelerating;
        if (decelerationState == ControlledDeceleration.IDLE) {
            isDecelerating = true;
            decelerationState = ControlledDeceleration.DECELERATING;
            decelerationStart = startSpeed;
            decelerationSide = driveDirection;
            decelerationCurrent = startSpeed;
        } else {
            isDecelerating = true;
        }

        return isDecelerating;
    }

    public void stopDecelerating() {
        if (decelerationState != ControlledDeceleration.IDLE) {
            decelerationState = ControlledDeceleration.IDLE;
            setAllDriveZero();
        }
    }

    public void performDeceleration() {
        switch(decelerationState) {
            case DECELERATING:
                decelerationCurrent -= ACCEL_STEP;
                if(decelerationCurrent < 0.0) {
                    decelerationCurrent = 0.0;
                    decelerationState = ControlledDeceleration.IDLE;
                }
                switch(decelerationSide) {
                    case LEFT:
                        drive(-decelerationCurrent, 0, 0, -readIMU(), false);
                        break;
                    case RIGHT:
                        drive(decelerationCurrent, 0, 0, -readIMU(), false);
                        break;
                    case BACK:
                        drive(0, -decelerationCurrent, 0, -readIMU(), false);
                        break;
                    case FRONT:
                        drive(0, decelerationCurrent, 0, -readIMU(), false);
                        break;
                }
                break;
            case STOPPING:
                setAllDriveZero();
                decelerationState = ControlledDeceleration.IDLE;
            case IDLE:
                break;
        }
    }

    public boolean startGrabbing() {
        boolean isGrabbing;
        if (grabState == GrabFoundationActivity.IDLE) {
            isGrabbing = true;
            grabState = GrabFoundationActivity.BACKING_UP;
            gotoRearTarget(0.3, 0.1);
        } else {
            isGrabbing = true;
        }

        return isGrabbing;
    }

    public void stopGrabbing() {
        if (grabState != GrabFoundationActivity.IDLE) {
            grabState = GrabFoundationActivity.IDLE;
            setAllDriveZero();
        }
    }

    public void performGrabbing() {
        switch(grabState)
        {
            case GRABBING:
                if((fingerTimer.milliseconds() >= FINGER_ROTATE_TIME)) {
                    setAllDriveZero();
                    grabState = GrabFoundationActivity.IDLE;
                }
                break;
            case BACKING_UP:
                if(gotoRearTarget(0.3, 0.1)) {
                    grabState = GrabFoundationActivity.GRABBING;
                    fingersDown();
                    fingerTimer.reset();
                }
                break;
            case STOPPING:
                setAllDriveZero();
                grabState = GrabFoundationActivity.IDLE;
            case IDLE:
            default:
                break;
        }
    }

    public boolean startFoundation() {
        boolean isGrabbing;
        if (removeFoundation == FoundationActivities.IDLE) {
            isGrabbing = true;
            removeFoundation = FoundationActivities.GRAB;
            startGrabbing();
        } else {
            isGrabbing = true;
        }

        return isGrabbing;
    }

    public void stopFoundation() {
        if (removeFoundation != FoundationActivities.IDLE) {
            removeFoundation = FoundationActivities.IDLE;
            stopGrabbing();
            stopAccelerating();
            stopDecelerating();
            fingersUp();
            setAllDriveZero();
        }
    }

    public void performFoundation() {
        switch(removeFoundation)
        {
            case STOP:
                if(fingerTimer.milliseconds() >= FINGER_ROTATE_TIME) {
                    removeFoundation = FoundationActivities.IDLE;
                }
                break;
            case DECEL:
                if(decelerationState == ControlledDeceleration.IDLE) {
                    fingersUp();
                    fingerTimer.reset();
                    removeFoundation = FoundationActivities.STOP;
                }
                break;
            case ACCEL:
                if(accelerationState == ControlledAcceleration.IDLE) {
                    startDecelerating(0.5, RobotSide.FRONT);
                    removeFoundation = FoundationActivities.DECEL;
                }
                break;
            case SETTLE:
                if(settleTimer.milliseconds() >= 500) {
                    startAccelerating(0.5, RobotSide.FRONT);
                    removeFoundation = FoundationActivities.ACCEL;
                }
                break;
            case GRAB:
                if(grabState == GrabFoundationActivity.IDLE) {
                    settleTimer.reset();
                    removeFoundation = FoundationActivities.SETTLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    public boolean startStoneStacking() {
        boolean isStacking;
        if (stackStone == StackActivities.IDLE) {
            isStacking = true;
            stackStone = StackActivities.LIFT;
            startLifting();
        } else {
            isStacking = true;
        }

        return isStacking;
    }

    public void performStoneStacking() {
        switch(stackStone)
        {
            case STOW:
                if(stowState == StowActivity.IDLE) {
                    stackStone = StackActivities.IDLE;
                }
                break;
            case RELEASE:
                if(releaseState == ReleaseActivity.IDLE) {
                    stackStone = StackActivities.STOW;
                    startStowing();
                }
                break;
            case LIFT:
                if(liftState == LiftActivity.IDLE) {
                    stackStone = StackActivities.RELEASE;
                    startReleasing();
                }
                break;
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
                lifterSpeed = LIFT_MAX_SPEED;
            } else {
                if(stowingLift) {
                    lifterSpeed = LIFT_MID_SPEED;
                } else {
                    lifterSpeed = LIFT_MIN_SPEED;
                }
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

    public int getLifterAbsoluteEncoder() {
        if(!hub2Read) {
            bulkDataHub2 = expansionHub2.getBulkInputData();
            hub2Read = true;
        }
        lifterEncoderValue = bulkDataHub2.getMotorCurrentPosition(lifter);

        return lifterEncoderValue;
    }

    public int getLifterPosition() {
        getLifterAbsoluteEncoder();

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

    public int getIntakeAbsoluteEncoder() {
        if(!hub1Read) {
            bulkDataHub1 = expansionHub1.getBulkInputData();
            hub1Read = true;
        }
        extenderEncoderValue = bulkDataHub1.getMotorCurrentPosition(extender);

        return extenderEncoderValue;
    }

    public int getIntakePosition() {
        getIntakeAbsoluteEncoder();

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
        // FrontLeft, FrontRight, Extender, LeftIntake
        expansionHub1 = hwMap.get(ExpansionHubEx.class, HUB1);
        // RearRight, RearLeft, Lifter, RightIntake
        expansionHub2 = hwMap.get(ExpansionHubEx.class, HUB2);


        clawTimer = new ElapsedTime();
        clawdricopterTimer = new ElapsedTime();
        fingerTimer = new ElapsedTime();
        settleTimer = new ElapsedTime();
        rightFinger = hwMap.get(Servo.class, RIGHT_FINGER);
        leftFinger = hwMap.get(Servo.class, LEFT_FINGER);
        claw = hwMap.get(Servo.class, CLAW);
        clawdricopter = hwMap.get(Servo.class, CLAWDRICTOPTER);
        leftIntake = (ExpansionHubMotor) hwMap.dcMotor.get(LEFT_INTAKE);
        rightIntake = (ExpansionHubMotor) hwMap.dcMotor.get(RIGHT_INTAKE);
        lifter = (ExpansionHubMotor) hwMap.dcMotor.get(LIFTER);
        extender = (ExpansionHubMotor) hwMap.dcMotor.get(EXTENDER);

        rightTof = (Rev2mDistanceSensor)hwMap.get(DistanceSensor.class, RIGHT_RANGE);
        leftTof = (Rev2mDistanceSensor)hwMap.get(DistanceSensor.class, LEFT_RANGE);
        backRightTof = (Rev2mDistanceSensor)hwMap.get(DistanceSensor.class, BACK_RIGHT_RANGE);
        backLeftTof = (Rev2mDistanceSensor)hwMap.get(DistanceSensor.class, BACK_LEFT_RANGE);
        backTof = (Rev2mDistanceSensor)hwMap.get(DistanceSensor.class, BACK_RANGE);
//        if(!tofInitialized) {
//            rightTof.initVL53L0X(false);
//            leftTof.initVL53L0X(false);
//            backRightTof.initVL53L0X(false);
//            backLeftTof.initVL53L0X(false);
//            backTof.initVL53L0X(false);
//            tofInitialized = true;
//        }

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
}

