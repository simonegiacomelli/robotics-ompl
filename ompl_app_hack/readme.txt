In folder 'gui' you will find the following files:
    * ompl_app.py it is the original file, you don't need this. It is here only as reference to see the
      difference with the following file
    * ompl_app_custom.py the only modification is this: when you click plan, it calls plugin1.configure(setup)
      in order to configure the robot "Kinematic Car". You can compare this file with the above to see the
      differences
    * plugin1.py there configuration(setup) that will set the propagator (for a custom kinematics)


In folder 'src/omplapp/apps' you will find:
    * KinematicCarPlanning.h this have a very small addition that allow us to expose the SpaceInformation
      in order to be able to change the propagator
    * KinematicCarPlanning_orig.h just to be able to verify the differences


In folder '_root' you will find:
    * install-partial.sh this will only recompile (no checks, no source download, no dependencies etc)

you need to copy (and possibly overwrite) only these files in the true ompl source tree:
* ompl_app_custom.py
* plugin1.py
* KinematicCarPlanning.h

Then copy 'install-partial.sh' close to 'install-ompl-ubuntu.sh' and run it from the terminal