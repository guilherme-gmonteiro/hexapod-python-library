Attempt to convert the hexapod-library from mithi to python

# Hexapod Kinematics Library

<p align="center">
    <img src="https://mithi.github.io/robotics-blog/show-off-v2-4.gif" alt="drawing" width="400" />
</p>

Code you can use to solve forward/inverse kinematics and generate walk sequences of hexapod robots. The codebase is largely copied from [Mithi's Bare Minimum Hexapod Robot Simulator 2](https://github.com/mithi/hexapod). [![Commit snapshot](https://img.shields.io/badge/commit%20snapshot-467d1a3b9-orange.svg?color=purple)](https://github.com/mithi/hexapod/tree/467d1a3b92dabd0304c7ef4675d64179f82efb69/src/hexapod)

Docs are written in the files themselves:

-   [`VirtualHexapod`](./src/VirtualHexapod.py)
-   [`getWalkSequence`](./src/solvers/walkSequenceSolver.py)
-   [`solveInverseKinematics`](./src/solvers/ik/hexapodSolver.py)

You can also inspect the [test directory](https://github.com/mithi/hexapod-kinematics-library/tree/main/tests) to see examples of how to use.

## Contributing [![PRs welcome!](https://img.shields.io/badge/PRs-welcome-orange.svg?style=flat)](https://github.com/mithi/mithi/wiki/Contributing)

Please read the [contributing guidelines](https://github.com/mithi/mithi/wiki/Contributing) and the recommended [commit style guide](https://github.com/mithi/mithi/wiki/Commit-style-guide)! Thanks!