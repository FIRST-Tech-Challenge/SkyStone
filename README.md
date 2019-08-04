## NOTICE

This repository contains v5.0 of the FTC SDK.  No further versions will pushed to https://github.com/ftctechnh/ftc_app.

v5.0 has also been posted to https://github.com/FIRST-Tech-Challenge/SkyStone where all further releases will be posted for the 2019/20 season.

Why are we doing this?  Git/GitHub is not designed to store large binary blobs.  As such the ftc_app repository has grown to over 1GB.  Not only does this violate GitHub's usage policies, but puts an undue burden on users of the repository when they download it.  For the foreseeable future a season specific version of the repository will be hosted at the url above.  Different approaches to prevent the size problem from recurring are being evaluated by the FTC Technology Team.  We realize that this creates a problem with teams that have maintained a history of software within the ftc_app workspace.  We appreciate your patience and understanding as work to migrate to a permanent solution.

## Welcome!
This GitHub repository contains the source code that is used to build an Android app to control a *FIRST* Tech Challenge competition robot.  To use this SDK, download/clone the entire project to your local computer.

If you are new to the *FIRST* Tech Challenge software and control system, you should visit the online wiki to learn how to install, configure, and use the software and control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;https://github.com/ftctechnh/ftc_app/wiki

Note that the wiki is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

## Downloading the Project
It is important to note that this repository is large and can take a long time and use a lot of space to download. If you would like to save time and space, there are some options that you can choose to download only the most current version of the Android project folder:

* If you are a git user, *FIRST* recommends that you use the --depth command line argument to only clone the most current version of the repository:

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone --depth=1 https://github.com/ftctechnh/ftc_app.git</p>

* Or, if you prefer, you can use the "Download Zip" button available through the main repository page.  Downloading the project as a .ZIP file will keep the size of the download manageable.

* You can also download the project folder (as a .zip or .tar.gz archive file) from the Downloads subsection of the Releases page for this repository.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains an online wiki with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access the wiki at the following address:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;https://github.com/ftctechnh/ftc_app/wiki

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Visit the following URL to view the FTC SDK documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;http://ftctechnh.github.io/ftc_app/doc/javadoc/index.html    

Documentation for the FTC SDK is also included with this repository.  There is a subfolder called "doc" which contains several subfolders:

 * The folder "apk" contains the .apk files for the FTC Driver Station and FTC Robot Controller apps.
 * The folder "javadoc" contains the JavaDoc user documentation for the FTC SDK.

### Online User Forum
For technical questions regarding the SDK, please visit the FTC Technology forum:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;http://ftcforum.usfirst.org/forumdisplay.php?156-FTC-Technology
