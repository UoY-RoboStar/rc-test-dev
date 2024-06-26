# Installation of RoboTool environment for development
These instructions are for a Linux environment, in particular Ubuntu LTS 20.04/22.04 (x64).

1. Download the latest RoboTool packaged release from the [robotool repository](https://github.com/UoY-RoboStar/robotool/releases).
2. Extract the archive.
3. If running under Wayland, start Eclipe from the command line ensuring it is running in X11 mode, to avoid potential incompatibilities,
   by setting the `GDK_BACKEND` environment variable to `X11`, that is, by using the command `GDK_BACKEND=X11 ./eclipse`. Otherwise, start Eclipse normally. To
   determine whether you're running under Wayland, you can query the value of the environment variable `XDG_SESSION_TYPE`, for
   example, by running the command `echo $XDG_SESSION_TYPE` in a terminal.

### Setting up RoboTool development environment for developing plug-ins
Following on from the steps above, then make sure you uninstall the plug-in you are developing. For example,
to work on the development of the `RoboChart Trace Generator` you should first uninstall via the following sequence of steps:

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

## RoboTool plug-ins
The following is a description of plug-ins required and used for: (1) calculating forbidden traces; and (2) 
generating test drivers, that is ROS nodes, that can be used to test ROS software nodes. Setting up of an
environment for working on their development is deferred to later in the document.

### Forbidden trace generation ([robochart-trace-gen](https://github.com/UoY-RoboStar/robochart-trace-gen))
This plug-in generates forbidden traces of a finite size from a RoboChart component, such as State Machines, 
Controllers and Modules. These are sequences of interaction that end in an event that is forbidden by the model.

The current implementation can only currently generate untimed forbidden traces. For this, it uses the CSP 
semantics automatically calculated by RoboTool and the FDR model-checker. FDR creates a graph representation of 
the underlying operational semantics of CSP, which is then used to calculate, at every state in the graph, 
what events are forbidden. To facilitate this calculation, the plug-in uses a version of CSP processes with a 
compression function called `diamond` that guarantees there are no `tau` (silent) transitions in the graph, 
thus making the implementation much simpler, at the cost, of potentially having to perform more calculations upfront.

Given the name of the desired RoboChart component, the plug-in outputs a file of the same name with an extension
`.rtest`, under the folder `test-gen` by default, containing a set of forbidden traces up to the length specified. The syntax of the traces follows that of CSPM expressions, and can be parsed by the plug-in
[robotest-textual](https://github.com/UoY-RoboStar/robotest-textual), described below.

#### Dependencies
This plug-in requires [FDR4](https://cocotec.io/fdr/) to be installed in a path known to the plug-in. 
In particular, it interfaces with FDR by loading a `fdr.jar` file, via JNI. FDR cannot be distributed 
standalone, so this dependency cannot be shipped with RoboTool.

#### Outstanding implementation issues
1. The plug-in relies on a fixed location for finding the `fdr.jar` as defined in its `MANIFEST.MF`. It should be possible to load this dynamically,
   possibly by allowing users at run-time to provide the location for FDR, as already captured by the `robochart-textual` plug-in options.
2. Use of JNI with `fdr.jar` seems to have issues with reclaiming back memory from FDR, even after it has been terminated and exited.

### Forbidden trace grammar and conversion ([robotest-textual](https://github.com/UoY-RoboStar/robotest-textual))
This plug-in implements an Xtext grammar that can parse the forbidden traces generated by [robochart-trace-gen](https://github.com/UoY-RoboStar/robochart-trace-gen/)
in files ending with the extension `.rtspec`. There are in fact two dialects supported for such files, a CSPM dialect, that has no direct 
knowledge of RoboChart components and types, and a RC dialect, that can be used to describe forbidden traces in terms of RoboChart model
elements. It is more concise and can capture more information, that just CSPM traces.

Note that from the Xtext grammar, an EMF metamodel is automatically generated. Instances of the metamodel are automatically 
populated when a file is parsed.

#### Converting between CSPM and RC traces
Conversion between forbidden trace formats is implemented by an [Epsilon](https://eclipse.dev/epsilon/) [ETL](https://eclipse.dev/epsilon/doc/etl/) 
model transformation program, that can be run from within RoboTool. This is useful, for example, given that the ROS test generator expects forbidden 
traces in the RoboChart dialect, rather than the CSPM form.

#### Outstanding implementation issues
1. The ETL transformation currently needs to be run manually from within Eclipse. It is possible to run such transformations programmatically,
   so ideally an integrated toolchain of these plug-ins should be able to make use of this workflow.
2. The ETL program has only been tested on a small set of examples. It is likely not currently comprehensive enough to cover all cases.

### ROS test generation ([robotest-ros-gen](https://github.com/UoY-RoboStar/robotest-ros-gen))
The ROS test generator currently targets [ROS Noetic](http://wiki.ros.org/noetic) and can support testing of ROS Nodes implementing a RoboChart component, such as
a state machine, making use of **topics** and **services**, but not **actions**. The generator works by generating C++ code: (1) a libary
that contains common code to a test suite, including some minimal amount of code to be provided by the user; (2) a main C++ program per
forbidden trace. It is integrated with [rostest](http://wiki.ros.org/rostest) via XUnit, but there is the possibility to enhance this integration, for example,
to allow parallelisation of testing, thus improving on the efficiency. 

#### Outstanding methodological issues
1. ROS1: Instrumentation of the NUT (Node Under Test) needs to be documented better and there is scope for providing a library of patterns,
or even explore approaches for implementing the same API, but for testing.
2. Considering timed tests.
3. Targetting of ROS2 should be more straightfoward given the ability to configure the DDS layer to deliver messages in order.

#### Outstanding implementation issues
1. The generated nodes have only been tested for a small subset of a case study on a Firegithing UAV. A comprehensive test suite for the
   tool is required, to provide further coverage and validation.
2. Actions are currently not supported, but since they are implemented via topics, this should be feasible, either by changing the generator
   or configuring the additional required topics.
4. No parallel tests at the moment via rostest. Could integrate with CMake for this.
