# MVGR
The Official Implementation for ''MVGR: Mean-Variance Minimization Global Registration Method for Multiview Point Cloud in Robot Inspection''

# Paper link: 
MVGR: Mean-Variance Minimization Global Registration Method for Multiview Point Cloud in Robot Inspection. [IEEE TIM 2024](https://ieeexplore.ieee.org/document/10565771)

# Abstract
The stitching and fusing of point clouds are indispensable for multiview 3-D measurement in robot inspection. Due to the positioning deviation of the robot, the point clouds collected by various poses inevitably have uneven density and stratification, leading to a significant compromise in the overall quality of multiview point clouds. This article proposes a mean-variance minimization global registration (MVGR) method, and the key idea is that the objective function is defined as the weighting of distance and variance between corresponding points. The constraint of variance helps mitigate the problem of uneven density. The perturbation model of Lie algebra se3 is utilized to derive the Jacobian matrix in the global optimization process of multiview point cloud poses. The iterative solution is obtained by the Newton-like iterative method. Furthermore, a point cloud slicing method is proposed to accelerate the search for corresponding points between point clouds. To demonstrate the superiority of MVGR, experiments are carried out and indicated that the point clouds registered by MVGR have minimum deviation compared with the previous SOTA methods, and the speed of searching for corresponding points can be enhanced by 50 times using a reasonable number of slices.

# Demo
You can run **demo.m** in Matlab to understand the implementation process of the method. Please note that this is just a simple implementation of the method in the paper.

![bunny-100](https://user-images.githubusercontent.com/39451786/201561500-bf13d4a4-3af1-4594-a300-3262136907fc.gif)






