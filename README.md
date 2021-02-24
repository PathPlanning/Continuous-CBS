# Continuous-CBS
Continuous CBS - a modification of conflict based search algorithm, that allows to perform actions of any arbitrary duration.
The main differences are the representation of constraints, timeline, collision detection mechanism and using SIPP algorithm as a low-level planner. More info about CCBS and its principles of work you can find at https://arxiv.org/abs/1901.05506

The master-version can work both on grids and roadmaps and supports the following enhancements:
- Disjoint Splitting
- Prioritizing conflicts
- High-level heuristics

The detailed descriptions of these enhancements can be found at https://arxiv.org/abs/2101.09723

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
git clone https://github.com/PathPlanning/Continuous-CBS.git
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
There are some options that can be controlled either through the `const.h` file or through the input configuration file (see examples):
* `<use_cardinal>` - controls whether the algorithm is looking for cardinal and semi-cardinal collisions or not. Possible values are `1`(true) or `0` (false).
* `<use_disjoint_splitting>` - controls whether the algotihm uses disjoint splitting enhancement or not. Possible values are `1`(true) or `0` (false).
* `<hlh_type>` - controls whether the algorithm uses high-level heuristics or not. 2 different heursitics are implemented. `0` - HL-heuristic is disabled; `1` - HL-heuristic based on solving linear programming problem (by simplex method); `2` - HL-heurstic based an greedy selection of disjoint cardinal conflicts. 
* `<connectedness>` - controls the connectedness of the grid. Possible values: `2` - 4 cardinal neighbors; `3` - 4 cardinal + 4 diagonal; `4` - 16 neighbors; `5` - 32 neighbors. In case if the map is represented as roadmap this parameter is ignored.
* `<timelimit>` - controls the maximum runtime of the algorithm. Possible values are >0. For example value 60 means that the algorithm can spend up to 60 seconds to find a solution.
* `<agent_size>` - controls the size (radii) of the agents' shape. Possible values are in the range (0, 0.5].
* `<precision>` - additional option, that controls how precise the end of collision interval is detected (the moment of time when there is no more collision between the agents). The lower the value - the preciser the algorithm finds the end of collision interval, but it takes a bit more time. Possible values are >0.

If some of these tags are not defined in the input configuraton file or there is no config-file, all the values of the absent parameters are taken from the 'const.h' file.

## Launch
To launch the application you need to have at least map and task input XML-files with all required information:
```
./CCBS map.xml task.xml
```
If you want to control the parameters through the input cofig file, you need to launch the application with three parameters:
```
./CCBS map.xml task.xml config.xml
```
The output file will be placed in the same folder as input files and, by default, will be named as task-file plus `_log.xml`. For examlpe,
```
"initial_task_file_name.xml" -> "initial_task_file_name_log.xml"
```

[![Build Status](https://travis-ci.org/PathPlanning/Continuous-CBS.svg?branch=master)](https://travis-ci.org/PathPlanning/Continuous-CBS)
