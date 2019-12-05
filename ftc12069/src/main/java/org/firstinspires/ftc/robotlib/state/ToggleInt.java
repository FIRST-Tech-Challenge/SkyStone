package org.firstinspires.ftc.robotlib.state;

//Makes menus possible
public class ToggleInt
{
    private int toggle;
    private boolean previouslyPressed = false;
    private final int maxToggle;

    public ToggleInt(int maxToggle)
    {
        this(maxToggle, 0);
    }

    public ToggleInt(int maxToggle, int startingToggle)
    {
        this.maxToggle = maxToggle;
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
        toggle++;
        toggle %= maxToggle;
    }

    public void setToggle(int toggle)
    {
        this.toggle = toggle;
    }

    public int output()
    {
        return toggle;
    }
}
