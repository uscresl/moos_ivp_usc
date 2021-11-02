# moos_ivp_usc
This repository is an extension to the MOOS-IvP repositories. 
You should install those first.

## What this repo contains
This is the full repository of moos_ivp_usc code tested between 2012 and 2018 at USC. 
Some of the processes and behaviors in this repo have been more cleanly shared already at 
* https://github.com/StephanieKemna/lib_behaviors-stephanie
* https://github.com/StephanieKemna/moos-ivp_processes_stephanie

I (Stephanie) refer to those for code that has been better tested to take stand-alone.

This repo is being shared to give access to the pGP and pGP_AUV processes, that were used for the publications related to Stephanie's PhD thesis, see https://stephaniekemna.github.io/#Publications <br/>
where:
* pGP: was last tested for the async surfacing strategies for multi-robot adaptive informative sampling (ref. Stephanie Kemna and Gaurav S. Sukhatme. "Surfacing strategies for multi-robot adaptive informative sampling with a surface-based data hub". In MTS/IEEE Oceans (Oceans), https://doi.org/10.1109/OCEANS.2018.8604896 Oct 2018.)
* pGP_AUV: was last tested on the EcoMapper AUV for the field testing (ref. Stephanie Kemna, Hordur K. Heidarsson and Gaurav S. Sukhatme. "On-board Adaptive Informative Sampling for AUVs: a Feasibility Study". In MTS/IEEE Oceans (Oceans), https://doi.org/10.1109/OCEANS.2018.8604838 Oct 2018.)

This code has not been maintained since. Feel free to create an 'issue' (preferably here: https://github.com/StephanieKemna/moos_ivp_usc for faster response), though a response/fix cannot be guaranteed.

## How to get MOOS-IvP and test both that and this repo

### Getting MOOS-IvP
* Go to: http://moos-ivp.org/ -- Download
* Do an svn checkout of the repositories
* Get the libraries listed in README-LINUX.txt
* build-moos
* build-ivp
* add env vars to .bashrc (add bin dirs to PATH, add lib_behaviors* to IVP_BEHAVIOR_DIRS)
* test that it works (e.g. run ivp/missions/m2_berta/, to quit: Ctrl+C)

### For this repo:
* get the desired dependencies (see dependencies.txt)
* run $ ./build.sh
* add the bin dirs to your PATH and lib_behaviors* to IVP_BEHAVIOR_DIRS
* test that it works 
  (e.g. run 
        $ pAntler xrelay.moos
        or run 
        $ pAntler simple-auv.moos )

## Original README file from moos-ivp-extend
```
##############################################################################
# FILE:        moos-ivp-extend/README
# DATE:        2011/09/07
# DESCRIPTION: Contains important information regarding the moos-ivp-extend
#              repository.
##############################################################################

#=============================================================================
# Introduction
#=============================================================================
The moos-ivp-extend repository contains examples for extending the MOOS-IvP
Autonomy system. This includes a MOOS application and an IvP behavior.


#=============================================================================
# Directory Structure
#=============================================================================
The directory structure for the moos-ivp-extend is decribed below:

bin              - Directory for generated executable files
build            - Directory for build object files
build.sh         - Script for building moos-ivp-extend
CMakeLists.txt   - CMake configuration file for the project
data             - Directory for storing data
docs             - Directory for storing documents
lib              - Directory for generated library files
missions         - Directory for mission files
README           - Contains helpful information - (this file).
scripts          - Directory for script files
src              - Directory for source code


#=============================================================================
# Build Instructions
#=============================================================================
#--------------------
# Linux and Mac Users
#--------------------

To build on Linux and Apple platforms, execute the build script within this
directory:

   $ ./build.sh

To build without using the supplied script, execute the following commands
within this directory:

   $ mkdir -p build
   $ cd build
   $ cmake ../
   $ make
   $ cd ..


#--------------
# Windows Users
#--------------
To build on Windows platform, open CMake using your favorite shortcut. Then 
set the source directory to be this directory and set the build directory
to the "build" directory inside this directory.

The source directory is typically next to the question:
   "Where is the source code?"

The build directory is typically next to the question:
   "Where to build the binaries?"

Alternatively, CMake can be invoked via the command line. However, you must
specify your gernerator. Use "cmake --help" for a list of generators and
additional help.

#=============================================================================
# Environment variables
#=============================================================================
The moos-ivp-extend binaries files should be added to your path to allow them
to be launched from pAntler. 

In order for generated IvP Behaviors to be recognized by the IvP Helm, you
should add the library directory to the "IVP_BEHAVIOR_DIRS" environment 
variable.

##############################################################################
#                               END of README
##############################################################################
```
