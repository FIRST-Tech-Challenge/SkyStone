/**
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1920;

import com.hfrobots.tnt.corelib.control.FakeOnOffButton;
import com.hfrobots.tnt.corelib.control.FakeRangeInput;
import com.hfrobots.tnt.corelib.drive.FakeExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.FakeServo;
import com.hfrobots.tnt.fakes.FakeHardwareMap;
import com.hfrobots.tnt.fakes.FakeNinjaGamePad;
import com.hfrobots.tnt.fakes.FakeTelemetry;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class DeliveryMechanismTest {
    private FakeHardwareMap hardwareMap;

    private FakeExtendedDcMotor liftMotor;

    private FakeExtendedDcMotor leftIntakeMotor;

    private FakeExtendedDcMotor rightIntakeMotor;

    private FakeServo fingerServo;

    private FakeServo wristServo;

    private FakeServo elbowServo;

    private FakeServo shoulderServo;

    private OperatorControls controls;

    private FakeNinjaGamePad gamepad;

    private DeliveryMechanism deliveryMechanism;

    @Before
    public void setUp() {
        hardwareMap = new FakeHardwareMap();

        liftMotor = new FakeExtendedDcMotor();
        leftIntakeMotor = new FakeExtendedDcMotor();
        rightIntakeMotor = new FakeExtendedDcMotor();

        fingerServo = new FakeServo();
        wristServo = new FakeServo();
        elbowServo = new FakeServo();
        shoulderServo = new FakeServo();

        hardwareMap.addDevice("liftMotor", liftMotor);
        hardwareMap.addDevice("leftIntakeMotor", leftIntakeMotor);
        hardwareMap.addDevice("rightIntakeMotor", rightIntakeMotor);

        hardwareMap.addDevice("fingerServo", fingerServo);
        hardwareMap.addDevice("wristServo", wristServo);
        hardwareMap.addDevice("elbowServo", elbowServo);
        hardwareMap.addDevice("shoulderServo", shoulderServo);

        FakeTelemetry telemetry = new FakeTelemetry();

        deliveryMechanism = new DeliveryMechanism(hardwareMap, telemetry);

        gamepad = new FakeNinjaGamePad();

        controls = OperatorControls.builder().operatorsGamepad(gamepad)
                .deliveryMechanism(deliveryMechanism)
                .build();
    }

    @Test
    public void testFingers() {
        gamepad.reset();
        FakeOnOffButton gripButton = (FakeOnOffButton) gamepad.getYButton();
        FakeOnOffButton ungripButton = (FakeOnOffButton) gamepad.getAButton();

        controls.periodicTask();

        Assert.assertEquals(DeliveryMechanism.FINGER_UNGRIP, fingerServo.getPosition(), 0.001);

        gripButton.setPressed(true);
        controls.periodicTask();
        Assert.assertEquals(DeliveryMechanism.FINGER_GRIP, fingerServo.getPosition(), 0.001);

        gripButton.setPressed(false);
        ungripButton.setPressed(true);
        controls.periodicTask();
        Assert.assertEquals(DeliveryMechanism.FINGER_UNGRIP, fingerServo.getPosition(), 0.001);
    }

    @Test
    public void testStateMachine() {
        gamepad.reset();
        FakeOnOffButton gripButton = (FakeOnOffButton) gamepad.getYButton();
        FakeOnOffButton ungripButton = (FakeOnOffButton) gamepad.getAButton();
        FakeRangeInput liftThrottle = (FakeRangeInput) gamepad.getRightStickY();
        FakeOnOffButton stowButton = (FakeOnOffButton) gamepad.getBButton();

        Assert.assertEquals(DeliveryMechanism.LoadingState.class.getSimpleName(),
                deliveryMechanism.getCurrentStateName());

        // -----------------------------------------------------------
        // Test transition to gripped
        // -----------------------------------------------------------

        {
            // run through one (possible) state transition
            deliveryMechanism.periodicTask();

            gripButton.setPressed(true);
            deliveryMechanism.periodicTask();

            // In correct state machine state
            Assert.assertEquals(DeliveryMechanism.LowGripState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            // Finger is gripped
            Assert.assertEquals(DeliveryMechanism.FINGER_GRIP, fingerServo.getPosition(), 0.001);
        }

        // -----------------------------------------------------------
        // Test transition to reload
        // -----------------------------------------------------------

        {
            gamepad.reset();

            ungripButton.setPressed(true);

            deliveryMechanism.periodicTask();

            // In correct state machine state
            Assert.assertEquals(DeliveryMechanism.LoadingState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            // Finger is un-gripped
            Assert.assertEquals(DeliveryMechanism.FINGER_UNGRIP, fingerServo.getPosition(), 0.001);

            gamepad.reset();
            deliveryMechanism.periodicTask(); // needed to debounce the button

            gripButton.setPressed(true);

            deliveryMechanism.periodicTask();

            // In correct state machine state
            Assert.assertEquals(DeliveryMechanism.LowGripState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            // Finger is gripped
            Assert.assertEquals(DeliveryMechanism.FINGER_GRIP, fingerServo.getPosition(), 0.001);
        }

        // -----------------------------------------------------------
        // Test transition to moving clear
        // -----------------------------------------------------------

        {
            gamepad.reset();

            liftThrottle.setCurrentPosition(-0.8F);

            deliveryMechanism.periodicTask();

            // Because operator commanded "up", we should be in the "Move clear state" now
            Assert.assertEquals(DeliveryMechanism.MoveClearState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            deliveryMechanism.periodicTask();

            // Lift is moving upwards
            Assert.assertTrue(liftMotor.getPower() > 0);
            // Finger is (still) gripped
            Assert.assertEquals(DeliveryMechanism.FINGER_GRIP, fingerServo.getPosition(), 0.001);

            Assert.assertEquals(DeliveryMechanism.MoveClearState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            // pretend the motor is running
            liftMotor.setCurrentPosistion(liftMotor.getCurrentPosition() + DeliveryMechanism.LIFT_CLEAR_SUPERSTRUCTURE_POS + 1);

            deliveryMechanism.periodicTask();

            Assert.assertEquals(DeliveryMechanism.ArmMovingState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            // FIXME: Attempt to place while moving, un-grip, stow, etc...
        }

        // -----------------------------------------------------------
        // Test transition to past clear and going up - arm moving state?
        // -----------------------------------------------------------

        {
            deliveryMechanism.periodicTask();

            Assert.assertEquals(DeliveryMechanism.ArmMovingState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            // Lift is still moving upwards
            Assert.assertTrue(liftMotor.getPower() > 0);
            // Finger is (still) gripped
            Assert.assertEquals(DeliveryMechanism.FINGER_GRIP, fingerServo.getPosition(), 0.001);

        }

        // -----------------------------------------------------------
        // Test transition to placing
        // -----------------------------------------------------------

        {
            gamepad.reset(); // "let go" of the lift throttle
            liftThrottle.setCurrentPosition(0);

            deliveryMechanism.periodicTask(); // one to transition
            deliveryMechanism.periodicTask(); // the other to actually do what the state does

            assertPlaceStateCommon();
        }

        // -----------------------------------------------------------
        // Test transition from moving to max
        // -----------------------------------------------------------

        {
            gamepad.reset(); // "let go" of the lift throttle
            liftThrottle.setCurrentPosition(-1);

            deliveryMechanism.periodicTask(); // one to transition
            deliveryMechanism.periodicTask(); // the other to actually do what the state does

            Assert.assertEquals(DeliveryMechanism.ArmMovingState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            // Lift is still moving upwards
            Assert.assertTrue(liftMotor.getPower() > 0);

            liftMotor.setCurrentPosistion(DeliveryMechanism.LIFT_MAX_HEIGHT_POS + 3);

            deliveryMechanism.periodicTask();
            deliveryMechanism.periodicTask();

            Assert.assertEquals(DeliveryMechanism.AtMaxState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());
            Assert.assertEquals( deliveryMechanism.LIFT_HOLD_FEED_FORWARD, liftMotor.getPower(), .0001);

            // Try and keep moving upwards, even though at max height
            liftThrottle.setCurrentPosition(-1);

            deliveryMechanism.periodicTask();
            deliveryMechanism.periodicTask();

            Assert.assertEquals(DeliveryMechanism.AtMaxState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());
            Assert.assertEquals( deliveryMechanism.LIFT_HOLD_FEED_FORWARD, liftMotor.getPower(), .0001);

        }

        // -----------------------------------------------------------
        // Test transition from max to min
        // -----------------------------------------------------------

        {
            gamepad.reset(); // "let go" of the lift throttle
            Assert.assertEquals(DeliveryMechanism.AtMaxState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            liftThrottle.setCurrentPosition(1);

            deliveryMechanism.periodicTask(); // one to transition
            deliveryMechanism.periodicTask(); // the other to actually do what the state does

            // Lift is still moving upwards
            Assert.assertTrue(liftMotor.getPower() < 0);

            liftMotor.setCurrentPosistion(DeliveryMechanism.LIFT_CLEAR_SUPERSTRUCTURE_POS -  3);

            deliveryMechanism.periodicTask();
            deliveryMechanism.periodicTask();

            Assert.assertEquals(DeliveryMechanism.AtMinState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());
            Assert.assertEquals( deliveryMechanism.LIFT_HOLD_FEED_FORWARD, liftMotor.getPower(), .0001);

            liftThrottle.setCurrentPosition(1);

            deliveryMechanism.periodicTask();
            deliveryMechanism.periodicTask();

            Assert.assertEquals(DeliveryMechanism.AtMinState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());
            Assert.assertEquals( deliveryMechanism.LIFT_HOLD_FEED_FORWARD, liftMotor.getPower(), .0001);

        }

        // -----------------------------------------------------------
        // Get lift moving upwards again
        // -----------------------------------------------------------

        {
            gamepad.reset(); // "let go" of the lift throttle
            Assert.assertEquals(DeliveryMechanism.AtMinState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            liftThrottle.setCurrentPosition(-1);

            deliveryMechanism.periodicTask(); // one to transition
            deliveryMechanism.periodicTask(); // the other to actually do what the state does

            // Lift is still moving upwards
            Assert.assertTrue(liftMotor.getPower() > 0);
            Assert.assertEquals(DeliveryMechanism.ArmMovingState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());
        }

        // -----------------------------------------------------------
        // Exercise arm, grip from place state
        // -----------------------------------------------------------

        {
            //gamepad.reset();

            // FIXME: "push" the button
            // someButton.setPressed(true);

            //deliveryMechanism.periodicTask();

            // FIXME: Wrong assertion!!!
            //Assert.assertEquals("Low grip state", deliveryMechanism.getCurrentStateName());
        }

        // -----------------------------------------------------------
        // Test transition to moving down
        // -----------------------------------------------------------

        {
            gamepad.reset();

            liftThrottle.setCurrentPosition(0.8F);
            liftMotor.setCurrentPosistion(liftMotor.getCurrentPosition() + 1000);

            deliveryMechanism.periodicTask();
            deliveryMechanism.periodicTask();

            Assert.assertEquals(DeliveryMechanism.ArmMovingState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());
        }

        // -----------------------------------------------------------
        // Test transition to at clear - "auto-stow?"
        // -----------------------------------------------------------

        {
            gamepad.reset();

            deliveryMechanism.periodicTask();

            assertPlaceStateCommon();

            liftMotor.setCurrentPosistion(DeliveryMechanism.LIFT_CLEAR_SUPERSTRUCTURE_POS + 100);

            stowButton.setPressed(true);

            deliveryMechanism.periodicTask();

            gamepad.reset();

            Assert.assertEquals(DeliveryMechanism.StowReturnState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());

            // Can't "see inside" the stow state machine right now
            for (int i = 0; i < 1000000; i++) {
                deliveryMechanism.periodicTask();

                if (Math.abs(shoulderServo.getPosition() - DeliveryMechanism.SHOULDER_STOW) <= 0.001) {
                    break;
                }
            }

            Assert.assertEquals(DeliveryMechanism.SHOULDER_STOW, shoulderServo.getPosition(), 0.001);

            // wait for shoulder to stow (delay states)

            long now = System.currentTimeMillis();

            while (System.currentTimeMillis() - now < 60000) {
                deliveryMechanism.periodicTask();

                if (liftMotor.getPower() < 0) {
                    break; // PID is moving the lift
                }
            }

            Assert.assertTrue(liftMotor.getPower() < 0);

            deliveryMechanism.periodicTask();
            Assert.assertTrue(liftMotor.getPower() < 0);

            liftMotor.setCurrentPosistion(DeliveryMechanism.LIFT_CLEAR_SUPERSTRUCTURE_POS / 2);
            deliveryMechanism.periodicTask();
            Assert.assertTrue(liftMotor.getPower() < 0);

            // reach bottom
            liftMotor.setCurrentPosistion(DeliveryMechanism.LIFT_MIN_HEIGHT_POS - 1);
            deliveryMechanism.periodicTask();
            Assert.assertEquals(0, liftMotor.getPower(), 0.001);

            Assert.assertEquals(DeliveryMechanism.LoadingState.class.getSimpleName(),
                    deliveryMechanism.getCurrentStateName());
        }

        // FIXME: Transitions left to test

        // atMin, arm out, grip, un-grip, go up (allowed), go down (not allowed), stow (allowed)

        // atMax, arm out, grip, un-grip, go up (not allowed) go down (allowed), stow (allowed)
    }

    private void assertPlaceStateCommon() {
        Assert.assertEquals(DeliveryMechanism.PlaceState.class.getSimpleName(),
                deliveryMechanism.getCurrentStateName());

        // Lift is held with feed forward upwards
        Assert.assertEquals(DeliveryMechanism.LIFT_HOLD_FEED_FORWARD, liftMotor.getPower(), .001);
        // Finger is (still) gripped
        Assert.assertEquals(DeliveryMechanism.FINGER_GRIP, fingerServo.getPosition(), 0.001);
    }
}