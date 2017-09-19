This repository is an extension to the MOOS-IvP repositories. 
You should install those first.

Getting MOOS-IvP
* Go to: http://moos-ivp.org/ -- Download
* Do an svn checkout of the repos
* Get the libraries listed in README-LINUX.txt
* build-moos
* build-ivp
* add env vars to .bashrc (add bin dirs to PATH, add lib_behaviors* to IVP_BEHAVIOR_DIRS)
* test that it works (e.g. run ivp/missions/m2_berta/, to quit: Ctrl+C)

For this repo:
* get the desired dependencies (see dependencies.txt)
* run $ ./build.sh
* add the bin dirs to your PATH and lib_behaviors* to IVP_BEHAVIOR_DIRS
* test that it works 
  (e.g. run 
        $ pAntler xrelay.moos
        or run 
        $ pAntler simple-auv.moos )
