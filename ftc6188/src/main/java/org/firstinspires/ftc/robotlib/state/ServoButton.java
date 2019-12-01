package org.firstinspires.ftc.robotlib.state;
/*
Operates like a button but accepts 3 inputs and returns a servo enum instead of a boolean
gives priority input in the following order (UP > DOWN > STOWED)
 */
public class ServoButton
{
    // the buttons used to determine up or down
    private Button upButton;
    private Button downButton;
    private Button stowedButton;

    public ServoButton()
    {
        upButton = new Button();
        downButton = new Button();
        stowedButton = new Button();
    }

    public void input(boolean upInput, boolean downInput, boolean stowedInput)
    {
        upButton.input(upInput);
        downButton.input(downInput);
        stowedButton.input(stowedInput);
    }

    public void input(boolean upInput, boolean downInput) { input(upInput, downInput, false); }

    public ServoState output()
    {
        ServoState returnState = ServoState.UNKNOWN;

        /*
        important that the if statement iterates through all buttons due to the nature of the button class
        if not all checked the button class retains that the button has not been pressed/released and would thus
        iterate the servo through the states in opposite order (just a guess TODO: TEST BUTTON CLASS FOR THIS INTERACTION ^
         */
        if (stowedButton.onPress() || stowedButton.onRelease()) { returnState = ServoState.STOWED; }

        if (downButton.onPress() || downButton.onRelease()) { returnState = ServoState.DOWN; }

        if (upButton.onPress() || upButton.onRelease()) { returnState = ServoState.UP; }

        return returnState;
    }
}
