package org.firstinspires.ftc.robotlib.state;

public class ToggleBoolean
{
    private boolean toggle = false;
    private boolean previouslyPressed = false;

    public ToggleBoolean()
    {

    }

    public ToggleBoolean(boolean startingToggle)
    {
        toggle = startingToggle;
    }

    public void input(boolean currentlyPressed)
    {
        if (currentlyPressed && !previouslyPressed)
        {
            toggle();
        }
        previouslyPressed = currentlyPressed;
    }

    public void toggle()
    {
        toggle = !toggle;
    }

    public void setToggle(boolean toggle)
    {
        this.toggle = toggle;
    }

    public boolean output()
    {
        return toggle;
    }
}
