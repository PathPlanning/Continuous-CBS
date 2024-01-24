# AA-CCBS
CCBS is a modification of the Conflict Based Search (CBS) algorithm, that supports actions (both move or wait) of arbitrary duration. CCBS is different from CBS in the way how conflicts and constraints are defined. To handle CCBS constraints the low-level search is inspired by Safe Interval Path Planning (SIPP) algorithm. More info about CCBS can be found at [IJCAI19 paper](https://www.ijcai.org/Proceedings/2019/0006.pdf).

This branch is a modification of CCBS adapted to solve any-angle variation of MAPF problem, when agents are allowed to move between any pair of grid cells if this movement doesn't cross any obstacles.
It's based on the combination of CCBS at the high-level and TO-AA-SIPP at the low-level. More details about TO-AA-SIPP can be found at [ICAPS21 paper](https://arxiv.org/pdf/2104.06681.pdf).

This version supports the following enhancements:
* Disjoint Splitting - an enhancement that imposes additional positive constraints to prevent creation of the same solution in different branches of the constraint tree.
* Multiconstraints - an enhancement that imposes constraints on multiple actions simultaneously.

This version doesn't support some of the enhancements made for original CCBS. It also doesn't support roadmaps and 2k-connected grids as this is an any-angle solver.

## Content

Content that compliments the source code of AA-CCBS is organized into the following folders:
* [Demos](https://github.com/PathPlanning/Continuous-CBS/tree/AA-CCBS/Demos) - contains animations of the solutions obtained by AA-CCBS.
* [Examples](https://github.com/PathPlanning/Continuous-CBS/tree/AA-CCBS/Examples) - contains the examples of input/output XML-files required/provided by the algorithm.
* [Instances](https://github.com/PathPlanning/Continuous-CBS/tree/AA-CCBS/Instances) - contains the archives with the maps and instances (in the format that AA-CCBS supports), that were used for the experimental evaluation described in the abovementioned papers on CCBS.
* [ExpResults](https://github.com/PathPlanning/Continuous-CBS/tree/AA-CCBS/ExpResults) - contains the raw tabular results obtained during the experimental evaluation of AA-CCBS algorithm.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

**[CMake](https://cmake.org/)** &mdash; an open-source, cross-platform family of tools designed to build, test and package software.

**[Boost](https://www.boost.org/)** &mdash; Boost provides free peer-reviewed portable C++ source libraries. Required due to the usage of multi-index.

### Installing

Download current repository to your local machine. Use
```
git clone https://github.com/PathPlanning/Continuous-CBS.git
```
or direct downloading.

Built current project using **CMake**. To launch the compiled file you will need to pass input XML file as an argument. Output file for this project will be placed in the same folder as input file and, by default, will be named `_log.xml`.
```bash
cd PATH_TO_THE_PROJECT
cmake .
make
```
## Input and Output files
The examples of input and output files you can find in the [Examples](https://github.com/PathPlanning/Continuous-CBS/tree/AA-CCBS/Examples) folder.

## Options
There are some options that can be controlled either through the `const.h` file or through the input configuration file (see [Examples](https://github.com/PathPlanning/Continuous-CBS/tree/AA-CCBS/Examples)):
* `<use_disjoint_splitting>` - controls whether the algotihm uses disjoint splitting enhancement or not. Possible values are `1`(true) or `0` (false).
* `<mc_type>` - controls whether the algorithm uses multiconstraints or not. `0` - multiconstraints disabled; `1` - naive version of multiconstraints that considers all posiible actions; `2` - a version that considers only the actions that lead to the cells crossed by the original movement action; `3` - additionally considers actions that starts from the cells crossed by the original action;
* `<focal_weight>` - controls the suboptimality factor of the found solutions. If value is `1` AA-CCBS looks for optimal solutions only, if `>1` - the solution might be suboptimal, but not more than the defined bound.
* `<timelimit>` - controls the maximum runtime of the algorithm. Possible values are >0. For example value 60 means that the algorithm can spend up to 60 seconds to find a solution.
* `<agent_size>` - controls the size (radii) of the agents' shape. Possible values are in the range (0, 0.5].
* `<precision>` - additional option, that controls how precise the end of collision interval is detected (the moment of time when there is no more collision between the agents). The lower the value - the preciser the algorithm finds the end of collision interval, but it takes a bit more time. Possible values are >0.

If some of these tags are not defined in the input configuraton file or there is no config-file, all the values of the absent parameters are taken from the 'const.h' file.

## Launch
To launch the application you need to have at least map and task input XML-files with all required information:
```
./AA-CCBS map.xml task.xml
```
If you want to control the parameters through the input config file, you need to launch the application with three parameters:
```
./AA-CCBS map.xml task.xml config.xml
```
The output file will be placed in the same folder as input files and, by default, will be named as task-file plus `_log.xml`. For examlpe,
```
"initial_task_file_name.xml" -> "initial_task_file_name_log.xml"
```
