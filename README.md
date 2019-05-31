# Robotics - Assignment 3 README
## Path planning with OMPL
## 31/05/2019
## Students: Simone Giacomelli, Oleg Borisov and Olivier Chapeau

### Video link
[Video](https://drive.google.com/file/d/1oe3lE8woQCAxYJaEnkaA3JKRpn9tbAl1/view)

---
### Folders content
- ompl_app_hack/_root:
    - install-partial.sh: recompiles the app (no checks, no source download, no dependencies etc)
- ompl_app_hack/src:
    - KinematicCarPlanning(.cpp/.h): Modified file with a very small addition that allow us to expose the SpaceInformation through python bindings; in order to be able to change the propagator
    - KinematicCarPlanning_orig(.cpp/.h): Original files
- ompl_app_hack/gui:
    - ompl_app_custom.py: Custom app file
    - plugin1.py: Main code file, the configure method is used to set the propagator (for a custom kinematics), the benchmark method is used to run benchmarks
- resources: Contains the meshes of the robots and the environments (.dae files) and the problems configuration files (.cfg files).
- benchmark_results: results of some benchmarks we run (databases and an additional pdf file)

---
### Setting up the environment
To setup the environment we followed the instructions given in the ompl [documentation installation page](https://ompl.kavrakilab.org/installation.html).
We downloaded the script and then changed its rights and executed it with the *--app* in order to install ompl with the python bindings and the ompl app. <br/>
**Note:** 6GB of RAM is mandatory in order to generate the python bindings correctly.
```console
$ chmod u+x install-ompl-ubuntu.sh
$ ./install-ompl-ubuntu.sh --app
```
#### Building the modified ompl_app
To build the custom ompl_app, you need to replace the files KinematicCarPlanning.h/.cpp in the source tree of the omplapp downloaded by the installation script (in omplapp-1.4.2/src/omplapp/apps) and then copy install-partial.sh near to install-ompl-ubuntu.sh and run it from the terminal.
```console
./install-partial.sh
```

---
### Running the modified ompl_app
Once built, the modified ompl_app can be launched by running the following command, in the ompl_app_hack/gui folder:
```console
python3 ompl_app_custom.py
```
### Or alternatively running the script plugin1.py
The planner can be launched directly without using the app, by calling the plugin1 file (in the ompl_app_hack/gui folder):
```console
python3 plugin1.py
```
Without options, the plugin1.py script launch the planner on the problem with the sticking nose. However the script plugin1.py can be runned with the following options:
* **--broken_wheel**: To run the problem with the broken wheel and without a stick nose
* **--bench**: To run the benchmarks on one of the previous problems
---
### Producing the benchmark plots
To produce the benchmarks, we tried to follow the instructions given in the [documentation](https://ompl.kavrakilab.org/benchmark.html), however some informations are outdated. Here are the steps to create visualizations from the benchmark log.
* First, you need to run the following command to generate the database
```console
ompl_benchmark_statistics.py logfile.log -d mydatabase.db
```
* Then, you have to go to the [planner arena](http://plannerarena.org/) website, part of the ompl project. In the menu there is an option "Change database" where you can upload the previously generated database.

* To finish, go to "Overall performance" in the menu of planner arena to have to tool to display different plots.

---
