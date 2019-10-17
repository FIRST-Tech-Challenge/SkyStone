package org.firstinspires.ftc.robotlib.state;

//This class stores the states for buttons pressed on the controller
public class Button
{
    private boolean previouslyPressed = false;
    private boolean pressed = false;
    private boolean onPress = false;
    private boolean onRelease = false;

    public void input(boolean currentlyPressed)
    {
        pressed = currentlyPressed;

        if (currentlyPressed && !previouslyPressed)
        {
            onPress = true;
        }

        if (!currentlyPressed && previouslyPressed)
        {
            onRelease = true;
        }

        previouslyPressed = currentlyPressed;
    }

    public boolean onPress()
    {
        if (onPress)
        {
            onPress = false;
            return true;
        }
        return false;
    }

    public boolean onRelease()
    {
        if (onRelease)
        {
            onRelease = false;
            return true;
        }
        return false;
    }

    public boolean isPressed()
    {
        return pressed;
    }
}
