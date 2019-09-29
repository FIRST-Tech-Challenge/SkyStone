# Example 1: Creating an OpMode
In this example you will:

 1. Copy a basic op mode from the FTC samples directory to your team directory.
 2. Modify the sample code and build it.
 4. Run it on the example robot.
 5. Commit your new op mode and push it to the repository.

## Step 0: Open Android Studio
You will need to open the FTC SkyStone project from Android Studio and let it sync with the project. This should only take a few seconds if you've opened the project previously.

## Step 1: Copy a basic op mode
The FTC project has many samples ready to be copied and modified to help you get started on writing code for your robot. There is a readme.md in the <code>FtcRobotController\src\main\java\org\firstinspires\ftc\robotcontroller\external\samples</code> directory in your cloned repository that outlines the naming convention and purposes of the various samples to help you find what you're looking for.

For this example we will be copying the BasicOpMode_Linear and modifying it.

 1. In the left hand pane that lists the project components (directories) click on the triangular arrow to expand **FTCRobotController** > **java** > **org.firstinspires.ftc.robotcotroller** > **external.samples**
 2. Right click on <code>BasicOpMode_Linear</code> and select **Copy**
 3. Expand your team project tree (in this case 6188) **ftc6188** > **java** > **org.firstinspires.ftc.teamcode**
 4. Right click on **org.firstinspires.ftc.teamcode** and select **Paste**
 5. A dialog will popup asking you to rename the class. Pick something obvious and unique. For example, **BasicTeleOp_*\<your name>***
 6. You will be prompted on whether you would like to add this new file to <code>git</code>. Click **Add**.

This is the basic workflow for copying any sample, either from the FTC samples or if the mentors put any in the <code>TeamCode</code> project.

## Step 2: Modify the op mode
Once you've copied the op mode and added it to the project, only the class has been renamed. There are several edits necessary to get it to work on the example robot.

 1. Remove the <code>@Disabled</code> line.
 2. Change the actual text that will display on the Driver Station. This is just plain text so put in any (short) description you think appropriate. Adding the team number will help differentiate which team's code is running.

> Change <code>@TeleOp(name=**"Basic: Linear OpMode"**, group="Linear Opmode")</code> to  <code>@TeleOp(name=**"\<your name> POV TeleOP (\<team number>) "**, group="Linear Opmode")</code>

 3. You will need to modify the hardware map (what the motors and sensors are named) in order to connect to the motors on the example robot.
 
> In the line `leftDrive = hardwareMap.get(DcMotor.class, "left_drive");` change `left_drive ` to `leftDrive`
> In the line `rightDrive = hardwareMap.get(DcMotor.class, "right_drive");` change `right_drive ` to `rightDrive`

 4. Hit **Ctrl+S** to save your changes.
 5. Select **Build > Make Project** to build your changes. The build should complete successfully.

## Step 3: Run the code on the robot
At this point you will need a phone with the correct configuration (**Example POV**) on it.

 1. Make sure the phone is on, booted, and unlocked.
 2. Plug the phone into the computer via USB.
 3. Wait for the computer and Android Studio to recognize the phone.
 
> You may need to wait a little while for this to complete. The phone may prompt you to allow the computer to attach to it if this the first time. You'll know it's complete when you see the phone listed in the devices dropdown in the toolbar (e.g. **zte N9130**).
 4. Ensure that your team specific run configuration is selected in the toolbar (e.g. **ftc6188**)
 5. Select **Run > Run \'ftc6188'**
