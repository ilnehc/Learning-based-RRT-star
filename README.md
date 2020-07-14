# Learning based Sampling for RRT* Algorithm
**Authors:** Li Chen, Yue Du, Yeyang Fang, Daiyao Yi, Xuran Zhao

This is the final project for EECS545 (Machine Learning) Winter 2020 at the University of Michigan, Ann Arbor.

Our report is located here: [report](https://github.com/ilnehc/Learning-based-RRT-star/blob/master/EECS545_Final_Report.pdf)

Our final presentation slides is located here: [slides](https://github.com/ilnehc/Learning-based-RRT-star/blob/master/Learning%20Based%20Sampling%20For%20RRT.pdf)

# 1. Overview
Compared with traditional uniformly sampled path planning approaches, a biased sampling method which learns from demonstrations and environmental restrictions would be preferable due to its efficiency. In this project, we have combined RRT* algorithm and a conditional variational autoencoder (CVAE) trained to learn from the latent dimensions and propose samples accordingly.

## CVAE
CVAE learns the distribution from the sampling points along the successful path. We refer to a sample point as X and external features as C. z denotes the latent variable. The architecture of the network is shown below:

<img src="https://github.com/ilnehc/Learning-based-RRT-star/blob/master/network.png" width="720">

## Biased sampling RRT* and bi-RRT*
We choose a greedy algorithm to sample from the results of CVAE. A typical result is shown below. You can refer to our report and slides for more info.

<img src="https://github.com/ilnehc/Learning-based-RRT-star/blob/master/complex_birrtstar_p_1.jpg" width="540">


# 2. Prerequisites
The Hybrid A* dataset generation code is on MATLAB. The CVAE training code is run on JupyterNotebook.

The RRT* and bi-RRT* experiment code is written by Python, which need libaries as follows:

1. Numpy.

2. Pygame. Look at How to install pygame for your environment: [pygame](https://www.pygame.org/wiki/GettingStarted).

  
## Acknowledgements
Our Hybrid A<sup>*</sup> code builds on [hybrid_A_star](https://github.com/wanghuohuo0716/hybrid_A_star) and RRT code builds on [RRT_Algorithms](https://github.com/vvrs/RRT_Algorithms).
