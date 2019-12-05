package org.firstinspires.ftc.robotlib.state;

//This class stores the states for buttons pressed on the controller
public class Button {
    private boolean previouslyToggled = false;
    private boolean toggled = false;
    private boolean isPressed = false;
    private boolean isReleased = false;

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

    public boolean isPressed()
    {
        if (isPressed)
        {
            isPressed = false;
            return true;
        }
        return false;
    }

    public boolean isReleased()
    {
        if (isReleased)
        {
            isReleased = false;
            return true;
        }
        return false;
    }

    public boolean isToggled()
    {
        return toggled;
    }
}