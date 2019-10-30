package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 *Created by Ethan
 */
public class HardwareOmnibot extends HardwareOmnibotDrive
{
    public enum LiftActivity {
        IDLE,
		LOWERING_TO_GRAB,
        GRABBING_STONE,
        CLEARING_LIFT,
        LIFTING_TO_ROTATE,
        ROTATING,
        LIFTING_TO_STONE,
        STOPPING
    }

    public enum ReleaseActivity {
        IDLE,
        ALIGN_TO_FOUNDATION,
        RELEASE_STONE,
        STOPPING
    }

    public enum StowActivity {
        IDLE,
        RAISING_TO_ROTATE,
        ROTATING,
        CLEARING_LIFT,
        LOWERING_TO_STOW,
		OPENING_CLAW,
        STOPPING
    }

    public enum EjectActivity {
        IDLE,
        EJECT,
        RESET,
        STOPPING
    }

    public enum CapstoneAlignActivity {
        IDLE,
        LOWERING_TO_ALIGN,
        OPENING_CLAW,
        ROTATING,
        LIFTING_TO_ROTATE,
        CLEARING_LIFT,
        STOPPING
    }

    public enum CapstoneGrabActivity {
        IDLE,
        LOWERING_TO_GRAB,
        GRABBING_CAPSTONE,
        LIFTING_TO_RUN,
        STOPPING
    }

    public enum CapstoneLiftActivity {
        IDLE,
        LIFTING_TO_STONE,
        STOPPING
    }

    public enum CapstoneReleaseActivity {
        IDLE,
        RELEASE_STONE,
        STOPPING
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
        STONE1(176),
        STONE2(474),
        STONE3(672),
        ROTATE(850),
        STONE4(907),
        STONE5(1131),
        STONE6(1357),
        STONE7(1593),
        STONE8(1810),
        STONE9(2030),
        STONE10(2251),
        STONE11(2475),
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
    public static double CLAW_OPEN = 0.2;
    public static double CLAW_PINCHED = 0.9;
    public static double CLAWDRICOPTER_FRONT = 0.85;
    public static double CLAWDRICOPTER_BACK = 0.09;
    public static int CLAW_OPEN_TIME = 1000;
    public static int CLAW_CLOSE_TIME = 1000;
    public static int CLAW_ROTATE_BACK_TIME = 1000;
    public static int CLAW_ROTATE_FRONT_TIME = 1000;
	private static int ENCODER_ERROR = 10;

    public LiftPosition liftTargetHeight = LiftPosition.STONE1;
	private LiftPosition liftStateTargetHeight = LiftPosition.STONE1;
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

    // Hardware objects
    protected Servo rightFinger = null;
    protected Servo leftFinger = null;
    protected Servo claw = null;
    protected Servo clawdricopter = null;
    protected DcMotor leftIntake = null;
    protected DcMotor rightIntake = null;
    protected DcMotor lifter = null;
    protected DcMotorEx extender = null;

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
    public CapstoneAlignActivity capstoneAlignState = CapstoneAlignActivity.IDLE;
    public CapstoneGrabActivity capstoneGrabState = CapstoneGrabActivity.IDLE;
    public CapstoneLiftActivity capstoneLiftState = CapstoneLiftActivity.IDLE;
    public CapstoneReleaseActivity capstoneReleaseState = CapstoneReleaseActivity.IDLE;
    private boolean fingersUp = true;
    private boolean clawPinched = false;
    private boolean clawdricopterBack = false;
    // These are the heights of the stone levels to auto set the lift to
    protected LiftPosition lastLiftHeight = LiftPosition.STOWED;
    protected int liftZero = 0;
    protected int intakeZero = 0;
    public double intakePower = 0.0;
    protected boolean intakeZeroUpdated = false;

	// Variables so we only read encoders once per loop
	protected boolean lifterEncoderRead = false;
	protected boolean extenderEncoderRead = false;
	protected int lifterEncoderValue = 0;
	protected int extenderEncoderValue = 0;

    /* Constructor */
    public HardwareOmnibot(){
        super();
    }

	public void resetEncoderReads() {
		lifterEncoderRead = false;
		extenderEncoderRead = false;
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

    public void startLifting() {
        if(liftState == LiftActivity.IDLE) {
            if(releaseState != ReleaseActivity.IDLE) {
                releaseState = ReleaseActivity.STOPPING;
            }
            if(stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
			// Extend the intake to make sure it isn't in the way.
			moveIntake(IntakePosition.EXTENDED);

			// We don't want the intake spinning while we are trying to lift the stone.
			stopIntake();
			liftState = LiftActivity.CLEARING_LIFT;
        }
    }

    public void performLifting() {
		switch(liftState) {
			case LIFTING_TO_STONE:
			    if(lifterAtPosition(liftStateTargetHeight)) {
					liftState = LiftActivity.IDLE;
				}
			    break;
		    case ROTATING:
			    if(stateTimer.milliseconds() >= CLAW_ROTATE_BACK_TIME) {
					liftState = LiftActivity.LIFTING_TO_STONE;
					clawdricopterBack = true;
					liftStateTargetHeight = liftTargetHeight;
					moveLift(liftStateTargetHeight);
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
                    liftState = LiftActivity.LIFTING_TO_ROTATE;
                    // If our target height is less than rotation height, we have to
                    // stop at rotation height first.
                    if(liftTargetHeight.getEncoderCount() <= LiftPosition.ROTATE.getEncoderCount()) {
                        moveLift(LiftPosition.ROTATE);
                    } else {
                        moveLift(liftTargetHeight);
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
			case STOPPING:
			    liftState = LiftActivity.IDLE;
		    case IDLE:
			default:
				break;
		}
    }

    public void startReleasing() {
        if(releaseState == ReleaseActivity.IDLE) {
            if(liftState != LiftActivity.IDLE) {
                liftState = LiftActivity.STOPPING;
            }
            if(stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
            releaseState = ReleaseActivity.RELEASE_STONE;
            claw.setPosition(CLAW_OPEN);
            stateTimer.reset();
        }
    }

    public void performReleasing() {
        switch(releaseState)
        {
            case ALIGN_TO_FOUNDATION:
                // In the future this should use distance sensors to line
                // up to the foundation.
                releaseState = ReleaseActivity.RELEASE_STONE;
                claw.setPosition(CLAW_OPEN);
                stateTimer.reset();
                break;
            case RELEASE_STONE:
                if((stateTimer.milliseconds() >= CLAW_OPEN_TIME) || !clawPinched)
                {
					clawPinched = false;
                    releaseState = ReleaseActivity.IDLE;
					// Add to our tower height for next lift.
					addStone();
                }
                break;
            case STOPPING:
                // I don't think we can do anything here. The servo going
                // either way is about the same.  Maybe when we implement
                // the ALIGN_TO_FOUNDATION
                releaseState = ReleaseActivity.IDLE;
            case IDLE:
		    default:
			    break;
        }
    }

    public void startStowing() {
        if(stowState == StowActivity.IDLE) {
            if(liftState != LiftActivity.IDLE) {
                liftState = LiftActivity.STOPPING;
            }
            if(releaseState != ReleaseActivity.IDLE) {
                releaseState = ReleaseActivity.STOPPING;
            }
			// Extend the intake to make sure it isn't in the way.
			moveIntake(IntakePosition.EXTENDED);

			// We don't want the intake spinning while we are trying to stow the
			// lift.
			stopIntake();

			// Start rotating back to the front.
            stowState = StowActivity.CLEARING_LIFT;
        }
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
						stateTimer.reset();
					}
				} else {
					stowState = StowActivity.ROTATING;
					claw.setPosition(CLAW_OPEN);
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
            case STOPPING:
                stowState = StowActivity.IDLE;
            case IDLE:
		    default:
			    break;
        }
    }

    public void startAligningCapstone() {
        if(capstoneGrabState == CapstoneGrabActivity.IDLE) {
            if(releaseState != ReleaseActivity.IDLE) {
                releaseState = ReleaseActivity.STOPPING;
            }
            if(stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
            // Extend the intake to make sure it isn't in the way.
            moveIntake(IntakePosition.EXTENDED);

            // We don't want the intake spinning while we are trying to lift the stone.
            stopIntake();
            capstoneAlignState = CapstoneAlignActivity.CLEARING_LIFT;
        }
    }

    public void performAligningCapstone() {
        switch(capstoneAlignState) {
            case LOWERING_TO_ALIGN:
                // Has it gotten low enough
                if(lifterAtPosition(LiftPosition.STOWED)) {
                    // Start grabbing the stone.  updateLifting will take over.
                    capstoneAlignState = CapstoneAlignActivity.IDLE;
                }
                break;
            case OPENING_CLAW:
                if((stateTimer.milliseconds() >= CLAW_OPEN_TIME) || !clawPinched) {
                    capstoneAlignState = CapstoneAlignActivity.LOWERING_TO_ALIGN;
                    clawPinched = false;
                }
                break;
            case ROTATING:
                if((stateTimer.milliseconds() >= CLAW_ROTATE_BACK_TIME) || clawdricopterBack) {
                    capstoneAlignState = CapstoneAlignActivity.LOWERING_TO_ALIGN;
                    clawdricopterBack = true;
                    moveLift(LiftPosition.STOWED);
                }
                break;
            case LIFTING_TO_ROTATE:
                // It has gotten high enough
                if(getLifterPosition() >= LiftPosition.ROTATE.getEncoderCount()) {
                    capstoneAlignState = CapstoneAlignActivity.ROTATING;
                    clawdricopter.setPosition(CLAWDRICOPTER_BACK);
                    claw.setPosition(CLAW_OPEN);
                    stateTimer.reset();
                }
                break;
            case CLEARING_LIFT:
                if(intakeAtPosition(IntakePosition.EXTENDED)) {
                    capstoneAlignState = CapstoneAlignActivity.LIFTING_TO_ROTATE;
                    moveLift(LiftPosition.ROTATE);
                }
                break;
            case STOPPING:
                capstoneAlignState = CapstoneAlignActivity.IDLE;
            case IDLE:
            default:
                break;
        }
    }

    public void startGrabbingCapstone() {
        if(capstoneGrabState == CapstoneGrabActivity.IDLE) {
            if(releaseState != ReleaseActivity.IDLE) {
                releaseState = ReleaseActivity.STOPPING;
            }
            if(stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
            moveLift(LiftPosition.GRABBING);
            capstoneGrabState = CapstoneGrabActivity.LOWERING_TO_GRAB;
        }
    }

    public void performGrabbingCapstone() {
        switch(capstoneGrabState) {
            case LIFTING_TO_RUN:
                if(lifterAtPosition(LiftPosition.STONE1)) {
                    capstoneGrabState = CapstoneGrabActivity.IDLE;
                }
                break;
            case GRABBING_CAPSTONE:
                if((stateTimer.milliseconds() >= CLAW_CLOSE_TIME) || clawPinched) {
                    capstoneGrabState = CapstoneGrabActivity.LIFTING_TO_RUN;
                    clawPinched = true;
                    moveLift(LiftPosition.STONE1);
                }
                break;
            case LOWERING_TO_GRAB:
                // It has gotten high enough
                if(lifterAtPosition(LiftPosition.GRABBING)) {
                    capstoneGrabState = CapstoneGrabActivity.GRABBING_CAPSTONE;
                    claw.setPosition(CLAW_PINCHED);
                    stateTimer.reset();
                }
                break;
            case STOPPING:
                capstoneGrabState = CapstoneGrabActivity.IDLE;
            case IDLE:
            default:
                break;
        }
    }

    public void startLiftingCapstone() {
        if(capstoneLiftState == CapstoneLiftActivity.IDLE) {
            if(releaseState != ReleaseActivity.IDLE) {
                releaseState = ReleaseActivity.STOPPING;
            }
            if(stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
            moveLift(liftTargetHeight);
            capstoneLiftState = CapstoneLiftActivity.LIFTING_TO_STONE;
        }
    }

    public void performLiftingCapstone() {
        switch(capstoneLiftState) {
            case LIFTING_TO_STONE:
                if(lifterAtPosition(liftTargetHeight)) {
                    capstoneLiftState = CapstoneLiftActivity.IDLE;
                }
                break;
            case STOPPING:
                capstoneGrabState = CapstoneGrabActivity.IDLE;
            case IDLE:
            default:
                break;
        }
    }

    public void startReleasingCapstone() {
        if(capstoneReleaseState == CapstoneReleaseActivity.IDLE) {
            if(releaseState != ReleaseActivity.IDLE) {
                releaseState = ReleaseActivity.STOPPING;
            }
            if(stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
            claw.setPosition(CLAW_OPEN);
            capstoneReleaseState = CapstoneReleaseActivity.RELEASE_STONE;
            stateTimer.reset();
        }
    }

    public void performReleasingCapstone() {
        switch(capstoneReleaseState) {
            case RELEASE_STONE:
                if((stateTimer.milliseconds() >= CLAW_OPEN_TIME) || !clawPinched) {
                    capstoneReleaseState = CapstoneReleaseActivity.IDLE;
                    clawPinched = false;
                }
                break;
            case STOPPING:
                capstoneReleaseState = CapstoneReleaseActivity.IDLE;
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

    public void startEjecting() {
        if (ejectState == EjectActivity.IDLE) {
            if (liftState != LiftActivity.IDLE) {
                liftState = LiftActivity.STOPPING;
            }
            if (stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
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

