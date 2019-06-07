# Continuous-CBS
Continuous CBS - a modification of conflict based search algorithm, that allows to perform actions of any arbitrary duration.

This branch is a version of Continiuous CBS that uses regular graphs instead of grids for environment description. It requires another format of input files. Instead of grid, input map-file contains a description of graph in GraphML notation. The task file should contain the IDs of nodes which correspond to the start and goal locations of the agents. More information about GraphML you can find at http://graphml.graphdrawing.org/

## Getting Started

To go and try this algorithm you can use QtCreator or CMake.
Both `.pro` and `CMakeLists` files are available in the repository.

Notice, that project uses C++11 standart. Make sure that your compiler supports it.

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

**[Qt Creator](https://info.qt.io/download-qt-for-device-creation?hsCtaTracking=c80600ba-f2ea-45ed-97ef-6949c1c4c236%7C643bd8f4-2c59-4c4c-ba1a-4aaa05b51086)**  &mdash; a cross-platform C++, JavaScript and QML integrated development environment which is part of the SDK for the Qt GUI Application development framework.

**[CMake](https://cmake.org/)** &mdash; an open-source, cross-platform family of tools designed to build, test and package software.

### Installing

Download current repository to your local machine. Use
```
git clone https://github.com/PathPlanning/AA-SIPP-m.git
```
or direct downloading.

Built current project using **Qt Creator** or **CMake**. To launch the compiled file you will need to pass input XML file as an argument. Output file for this project will be placed in the same folder as input file and, by default, will be named `_log.xml`. For examlpe, using CMake
```bash
cd PATH_TO_THE_PROJECT
cmake .
make
```
## Input and Output files
The examples of input and output files you can find in the Examples folder.

## Options
There are some options that can be controlled through the `const.h` file:
* CN_CARDINAL - controls whether the algorithm is looking for cardinal and semi-cardinal collisions or not. Possible values are `1`(true) or `0` (false).
* CN_HISTORY - controls whether the algorithm uses the information about previosly found collisions. This option allows to reduce the time requred for collision detection function, but it can't be directly combined with looking for cardinal conflicts. Possible values are `1`(true) or `0` (false).
* CN_STOP_CARDINAL - this option allows to stop looking for cardinal conflicts in cases, when the algorithm cannot find them anymore in the current branch of high-level tree. It also allows to combine CN_CARDINAL and CN_HISTORY options. Possible values are `1`(true) or `0` (false).
* CN_TIMELIMIT - controls the maximum runtime of the algorithm. Possible values are >0. For example CN_TIMELIMIT 60 means that the algorithm can spend up to 60 seconds to find a solution.
* CN_AGENT_SIZE - controls the size (radii) of the agents' shape. Possible values are >0.
* CN_DELTA - additional option, that controls the precision of detection of the end of collision interval (the moment of time when there is no more collision between the agents). The lower the value - the preciser the algorithm finds the end of collision interval, but it takes more time. Possible values are >0.

## Launch
To launch the application you need to have map and taks input XML-files with all required information:
```
./C-CBS map.xml task.xml
```
The output file will be placed in the same folder as input files and, by default, will be named as task-file plus `_log.xml`. For examlpe,
```
"initial_task_file_name.xml" -> "initial_task_file_name_log.xml"
```

[![Build Status](https://travis-ci.org/PathPlanning/Continuous-CBS.svg?branch=CCBS-graphs)](https://travis-ci.org/PathPlanning/Continuous-CBS)
