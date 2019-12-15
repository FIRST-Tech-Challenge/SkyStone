package org.firstinspires.ftc.robotlib.state;

/**
 * Represents a GamePad button with actions.
 */
public class Button {
    private boolean previouslyToggled = false;
    private boolean toggled = false;
    private boolean isPressed = false;
    private boolean isReleased = false;

    /**
     * Updates the state of the Button using an input.
     * @param currentlyToggled true if button is pressed
     */
    public void input(boolean currentlyToggled)
    {
        toggled = currentlyToggled;

        if (currentlyToggled && !previouslyToggled)
        {
            isPressed = true;
        }

        if (!currentlyToggled && previouslyToggled)
        {
            isReleased = true;
        }

        previouslyToggled = currentlyToggled;
    }

    /**
     * @return If the button has been pressed (was not toggled on last input)
     */
    public boolean isPressed()
    {
        if (isPressed)
        {
            isPressed = false;
            return true;
        }
        return false;
    }

    /**
     * @return If the button has been released (was toggled on last input)
     */
    public boolean isReleased()
    {
        if (isReleased)
        {
            isReleased = false;
            return true;
        }
        return false;
    }

    /**
     * @return If the button is down
     */
    public boolean isToggled()
    {
        return toggled;
    }
}