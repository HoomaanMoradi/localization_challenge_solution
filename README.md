# Robot Localization Challenge Solutions

This repository contains implementations of different localization algorithms for robot position estimation. The project includes two main approaches to solve the localization challenge: a Kalman Filter implementation and a Weighted Average approach.

## Project Structure

- `kalman_filter/`: Contains the implementation using Kalman Filter for state estimation
  - `main.cpp`: Main implementation file for the Kalman Filter solution
  - `controller.h`: Controller interface definition
  - `robot_defs.h`: Robot-related definitions and structures
  - `sample_input*.txt`: Sample input files for testing
  - `Makefile`: Build configuration
  - `Figures.txt`: Output figures and results
  - `Localization_Challenge.pdf`: Problem statement and requirements

- `weighted_average/`: Contains the implementation using a Weighted Average approach
  - `main.cpp`: Main implementation file for the Weighted Average solution
  - `controller.h`: Controller interface definition
  - `robot_defs.h`: Robot-related definitions and structures
  - `sample_input*.txt`: Sample input files for testing
  - `Makefile`: Build configuration
  - `Figures.txt`: Output figures and results
  - `Localization_Challenge.pdf`: Problem statement and requirements

## Building the Project

Each implementation can be built separately using the provided Makefiles.

### Kalman Filter Implementation
```bash
cd kalman_filter
make
```

### Weighted Average Implementation
```bash
cd weighted_average
make
```

## Running the Tests

After building, you can run the localization tests using the provided sample input files:

```bash
./localization_test < sample_input1.txt
```

## Results

The `Figures.txt` files in each directory contain the output and results of the localization algorithms.

## Documentation

For detailed problem statements and requirements, please refer to the `Localization_Challenge.pdf` file in each directory.


