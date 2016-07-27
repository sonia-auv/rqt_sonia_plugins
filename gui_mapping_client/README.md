# Vision Client

This ROS Node provide a GUI for managing and testing Vision Server (vision_server)

It has been developped with Qt4 GUI library using Qt-Designer

## How to Compile Vision Client

### SONIA Dependencies

The SONIA Software architecture will require that some changes to your system.

If you did not follow the [SONIA Wiki](http://sonia.etsmtl.ca:120/soniapedia/index.php/Proc%C3%A9dure_d%27installation),
please read it before trying to do anything.

Here are the main step you should follow :

1. Be sure to have the Vitals library compiled and installed on your machine.

	Visit the SoniaWiki for the How-To section about vitals

2. Be sure to have the vision_server project in your ROS package alongside
	VisionServer package

	It's in the same git branch, therefore, you should probably already have it

3. VisionClient will not compiled if you do not compile vision_server before.

	In order to do this, please run `catkin_make` once then
	`source $SONIA_WORKSPACE_ROOT/ros/devel/setup.bash`
	 and once again `catkin_make`

4. Be sure to have `$SONIA_WORKSPACE_ROOT/vitals`
 in your `LD_LIBRARY_PATH` variable

	To do so, please run
	`export LD_LIBRARY_PATH=$SONIA_WORKSPACE_ROOT/vitals:$LD_LIBRARY_PATH`
	or add it to your `~/.bashrc`

### Compile Qt-Designer UI

You have to generate a header file for your GUI, which is not atomatically done.

In order to do this, please use this command line:

```
uic-qt4 $SONIA_WORKSPACE_ROOT/ros/src/vision_client/src/your_ui_file.ui -o \
$SONIA_WORKSPACE_ROOT/ros/src/vision_client/include/ui_your_ui_file.h
```

The main window UI file is MainWindow.ui, to compile it, execute this command
line:

```
cd $SONIA_WORKSPACE_ROOT/ros/src/vision_client/ && \
uic-qt4 src/MainWindow.ui -o include/ui_MainWindow.h && \
cd -
```

### Compile the whole project

Finally, just execute a `catkin_make` in your `$SONIA_WORKSPACE_ROOT/ros`
directory:

```
cd $SONIA_WORKSPACE_ROOT/ros/ && \
source devel/setup.bash && \
catkin_make && \
cd -
```

## How to Create an Eclipse project

You can create the eclipse project by following these step

```
cd $SONIA_WORKSPACE_ROOT/ros/src/vision_client
mkdir project
cd project
cmake ../ -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
```

You are now ready to add a new project to your eclipse directory and link it to
`$SONIA_WORKSPACE_ROOT/ros/src/vision_client/project/`

## How to run Vision Client

Vision Client is on the node vision_client_node on ROS. To launch the thing,
simply run:

```
cd $SONIA_WORKSPACE_ROOT/ros/ && \
source devel/setup.bash && \
rosrun vision_client vision_client_node && \
cd -
```
