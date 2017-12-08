
## Project Introduction
The robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I implemented a 2 dimensional particle filter in C++. The particle filter was given a map and some initial localization information (analogous to what a GPS would provide). At each time step it would also get observation and control data. 

## Results

### Result of final submission ([code](https://github.com/leros/Self-Driving-Car-Kidnapped-Vehicle-Project/blob/2b247d64ffe2afe7088213d7c8146e2e5b73861c/src/particle_filter.cpp))
![](./assets/images/final_submissions.png)

### Fix a data type bug and cut the errors([code](https://github.com/leros/Self-Driving-Car-Kidnapped-Vehicle-Project/commit/2e2dd03c71278fb8bbae16bb1c4a5160edaf7db2))
![](./assets/images/fix_a_data_type_bug.png)

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
├── CMakeLists.txt
├── README.md
├── assets
│   ├── images
│   │   ├── final_submissions.png
│   │   └── fix_a_data_type_bug.png
│   └── papers
│       └── README.md
├── build.sh
├── clean.sh
├── cmakepatch.txt
├── data
│   └── map_data.txt
├── install-mac.sh
├── install-ubuntu.sh
├── run.sh
└── src
    ├── helper_functions.h
    ├── json.hpp
    ├── main.cpp
    ├── map.h
    ├── particle_filter.cpp
    └── particle_filter.h
```





