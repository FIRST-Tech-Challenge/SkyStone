# Welcome!
This GitHub repository contains the source code for Mount Si Robotics club's SkyStone *FIRST* Tech Challenge competition robot. We will be using a single, competition specific (i.e. SkyStone) repo which will contain all the code for all teams and samples. Below is a step-by-step guide to getting set up to work on code in this project

## Step 0: Get git
We will be using git to manage our source code. You should download and install the latest version of Git for Windows:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Git for Windows](https://git-scm.com/download/win)

This should start the download automatically. Once it has completed downloading open/run it.

1. Hit 'Next>' on the *Information* screen.
2. On *Select Components* you can leave the defaults although 'Use a TrueType font in all console windows' can be nice to have.
3. On *Choosing the default editor...* you can leave the default (Nano).
4. On *Adjusting yout PATH...* you should leave the default 'Git from the command line...'.
5. On *Choosing HTTPS...* leave the first option selected 'Use the OpenSSL library'.
6. On *Configuring the line ending...* please select the last option, 'Checkout as-is, commit as-is'.
7. On *Configuring the terminal...* leave the default 'Use Windows' default console window'.
8. On *Configuring extra options* leave the defaults checked.
9. On *Configuring experimental options* Do not opt-in. Leave it unchecked.
10. Hit 'Install'
11. When it's done you can uncheck 'View Release Notes' and hit 'Finish'.

Congratulations! You now have git!

## Step 1: Clone this repo

1. Open a command prompt.
2. Change to your root directory (EX: <code>cd c:\\</code>)
3. Make a directory for your repos (EX: <code>md repos</code>)
4. Change into your repos directory (EX: <code>cd repos</code>)
5. Make your directory for Mount Si repos (EX: <code>md mount-si</code>)
6. Change into your Mount Si repos directory (EX: <code>cd mount-si</code>)
7. Clone the Mount Si SkyStone repo onto your computer <code>git clone https://github.com/mount-si/SKYSTONE.git</code>

Well done! Now you have all our code on your computer!

## Step 2: Install Android Studio

1. Go to the [Android Studio download](https://developer.android.com/studio) 
2. Click on 'Download Android Studio'
3. When the download completes (it may take a while) open/run the installer. You can leave all the defaults.

I am sorry... you now have Android Studio on your computer. Sacrifices must be made...

## Step 3: Open and build our project

1. Open Andriod Studio
2. When asked, select that you would like to open an existing project
3. Navigate to the directory you cloned this repo into (EX: <code>c:\\repos\\mount-si\\SkyStone</code>)
4. Once the project is open it will run a Sync with Gradle. It should complete successfully.
5. Select 'Build' => 'Rebuild Project'. This should complete successfully.

That's it! You now have a completely functional development environment to start writing code.

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[SKYSTONE Online Documentation](https://github.com/FIRST-Tech-Challenge/SKYSTONE/wiki)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://first-tech-challenge.github.io/SkyStone/doc/javadoc/index.html)    

Documentation for the FTC SDK is also included with this repository.  There is a subfolder called "doc" which contains several subfolders:

 * The folder "apk" contains the .apk files for the FTC Driver Station and FTC Robot Controller apps.
 * The folder "javadoc" contains the JavaDoc user documentation for the FTC SDK.

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FTC Technology forum:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Technology Forum](https://ftcforum.usfirst.org/forumdisplay.php?156-FTC-Technology)

For any FTC questions including technical/programming, please visit the FTC Reddit:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Reddit](https://www.reddit.com/r/FTC/)