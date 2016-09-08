# About
This project contains the tools necessary for testing the contour-based multimodal registration algorithm.

# Set-up
The code provided contains a Dockefile containing all the necessay dependencies to compile the project. 
Scripts `build.sh` and `run.sh` are also provided to buld the image and run a container with the specified images. 
Any environment variable stating with `PHD_*` will be autmatically added to the container. Please configure your 
datasets and results folder in the `run.sh` script.

# Provided tests
This software is shipped jointly with a demo test of the algorithm. For running this test you will need the Stanford 
bunny data, both an static image of the bunny and a 3-D reconstuction. This test can be found into the `autorun` folder. 
To run this test, run `python autorun/runners.py fullpath` on your machine. Additionally, a test on the color gradient 
algorithm is also provided. To run this test, run `python autorun/runners.py gradientTestSynth`.

You can find the steps done on each test in the scripts `autorun/fullpath.py` and `autorun/gradientTestSynth`.

