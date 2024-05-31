# rc-test-dev
Repository with documentation regarding development of tests from RoboChart targetting ROS and similar environments.

## Installation of RoboTool environment for development
These instructions are for a Linux environment, in particular Ubuntu LTS 22.04 (x64).

1. Download the latest RoboTool packaged release from the [robotool repository](https://github.com/UoY-RoboStar/robotool/releases).
2. Extract the archive.
3. If running under Wayland, start Eclipe from the command line ensuring it is run in X11 mode by prepending setting the `GDK_BACKEND`
   environment variable to `x11`, that is, by using the command `GDK_BACKEND=x11 ./eclipse`. Otherwise, start Eclipse normally. To
   determine whether you're running under Wayland, you can query the value of the environment variable `XDG_SESSION_TYPE`, for
   example, by running the command `echo $XDG_SESSION_TYPE` in a terminal.

## RoboTool plug-ins
The following is a description of plug-ins required and used for: (1) calculating forbidden traces; and (2) 
generating test drivers, that is ROS nodes, that can be used to test ROS software nodes.

### Forbidden trace generation (robochart-trace-gen)
This plug-in generates forbidden traces of a finite size from a RoboChart component, such as State Machines, Controllers and Modules. These are
sequences of interaction that end in an event that is forbidden by the model.

The current implementation can only currently generate untimed forbidden traces. For this, it uses the CSP semantics automatically calculated by
RoboTool and the FDR model-checker. FDR creates a graph representation of the underlying operational semantics of CSP,
which is then used to calculate, at every state in the graph, what events are forbidden. To facilitate this calculation,
the plug-in uses a version of CSP processes with a compression function called `diamond` that guarantees there are no `tau`
(silent) transitions in the graph, thus making the implementation much simpler, at the cost, of potentially having to
perform more calculations upfront.

Given the name of the desired RoboChart component, the plug-in outputs a file of the same name with an extension `.rtest`, under the folder
`test-gen` by default, containing a set of forbidden traces up to the length specified. The syntax of the traces follows that of CSPM
expressions, and can be parsed by the plug-in [robotest-textual](https://github.com/UoY-RoboStar/robotest-textual), described below.

#### Dependencies
This plug-in requires FDR4 to be installed in a path known to the plug-in. In particular, it interfaces with FDR by
loading a `fdr.jar` file, via JNI. FDR cannot be distributed standalone, so this dependency cannot be shipped with RoboTool.

#### Outstanding implementation issues
1. The plug-in relies on a fixed location for finding the `fdr.jar` as defined in its `MANIFEST.MF`. It should be possible to load this dynamically,
   possibly by allowing users at run-time to provide the location for FDR, as already captured by the `robochart-textual` plug-in options.
2. Use of JNI with `fdr.jar` seems to have issues with reclaiming back memory from FDR, even after it has been terminated and exited.

### Forbidden trace grammar and conversion (robotest-textual)
This plug-in

### ROS component test generation (robotest-ros-gen)

### Setting up RoboTool development environment for developing plug-ins
Following on from the steps above, then:

1. Select `Help` > `About Eclipse IDE` > `Installation Details` and select the feature `RoboChart Trace Generator` in the
   `Installed Software` tab.
2. Uninstall the feature.
3. Click `Finish`.
4. Let Eclipse restart.

You're now ready to import the source Git repositories by following the instructions below.

1. Configure Eclipse to work with a local path for Git repositories by going to `Window` > `Preferences`, searching for `Git`
   and selecting it on the left-hand side. Change the `Default repository folder` as appropriate. Then click `Apply and Close`.
2. For each of the following repositories:
    * [robochart-trace-gen](https://github.com/UoY-RoboStar/robochart-trace-gen)
    * [robotest-textual](https://github.com/UoY-RoboStar/robotest-textual)
    * [robotest-ros-gen](https://github.com/UoY-RoboStar/robotest-ros-gen) (if working with ROS)

   Follow the following instructions:
    1. Open the repository on GitHub by following the link, then copy the SSH URL, e.g. clicking on `Code` and clicking on the
       clipboard copying icon.
    2. Then, go back to Eclipse. Select `File` > `Import`, and select `Git` > `Project from Git (with smart import)` and click `Next >`.
    3. Select `Clone URI` and click `Next >`.
    4. If you've copied the correct SSH URL to your clipboard the dialog should be automatically populated. Click `Next >`.
    5. You may now be prompted to authenticat to GitHub, preferably using the passphrase for your SSH private key. Enter it.
    6. By default at this stage Eclipse will select all branches for the chosen repository. Click `Next >`.
    7. Click `Next >` again, after which you'll be asked to chose which Eclipse projects to import.
    8. Import all projects. Before you do so, you may want to consider organising it by Java working sets (optional).
    9. Finally click `Next` and `Finish`. The projects should now show up in the `Package Explorer` or `Project Explorer`.
