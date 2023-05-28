# Robot-Mapping

Detailed development and implementation of:
- Discrete Counting Sensor Model (D-CSM); 
- Continuous Counting Sensor Model (C-CSM);
- Discrete Semantic Counting Sensor Model (D-SCSM) &
- Countinuous Semantic Counting Sensor Model (C-SCSM)


# Results

Model & Info        |  Map          |  Variance Map    
:-------------------------:|:-------------------------:|:-------------------------:
D-CSM 
Implemented a 2D counting sensor model (CSM) for occupancy grid mapping in **ogm_CSM.py**. Visualization of the map is with **grid_size = 0.135 m** and its associated variance map |  <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/a45a840c-2bbb-47ba-9d40-8f53179cbdc3">| <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/252d4c1c-230b-446f-80be-5eda9479523f">
C-CSM
Implemented a 2D continuous counting sensor model (CSM) in **ogm_continuous_CSM.py**. Visualization of the map is with **grid_size = 0.135 m** and its associated variance map |<img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/29afb8df-58a8-4ce1-83dd-acda47ee192f"> |<img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/7c94913e-8e90-44e9-8adc-163667198e18">
....... with **grid_size = 0.270 m** | <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/64ba25eb-6188-4b42-8275-1b71a385fb76"> | <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/69e8ddee-9e7a-46e9-abc8-8558adfe2e61">
....... with **grid_size = 0.500 m** | <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/f286576b-c253-40cc-b61a-e62b4ba412cf"> | <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/e4a38672-be6f-425a-8120-2e8ff4036f35">
D-SCSM
Implemented a semantic counting sensor model (S-CSM) in **ogm_S_CSM.py**. Visualization of the map is with **grid_size = 0.135 m** and variance map of the class with highest probabilities at each grid | <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/020d1474-d901-4d53-b9e0-e364c7b9c320"> | <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/c08f1e80-a5a8-41b1-9d45-e7898dae3d5c">
C-SCSM
Implemented a continuous semantic counting sensor model (S-CSM) in  **ogm_continuous_S_CSM.py**. Visualization of the map is with **grid_size = 0.135 m** and variance map of the class with highest probabilities at each grid | <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/6ce6f4cd-f89d-4fce-987b-f689be239075"> | <img width="111903" height="300" alt="image" src="https://github.com/DhyeyR-007/Robot-Mapping/assets/86003669/28949dbb-b0d7-489f-98fc-53da958a34e9">




## BKI Semantic Mapping Implementation

Build and run the [BKI Semantic Mapping](https://github.com/ganlumomo/BKISemanticMapping) on [KITTI semantics dataset sequence 04](http://www.cvlibs.net/datasets/kitti/eval_semantics.php). \
Notice the system dependencies:

- Ubuntu system. The mapping algorithm is build on Ubuntu system. It has been tested on Ubuntu
16.04 and Ubuntu 18.04. If you didn’t have an Ubuntu system, you could create a virtual machine.
- ROS system. The mapping algorithm has been tested on ROS kinetic and ROS melodic. You can
follow the installation guide for ROS in the documentation.
- A catkin workspace (catkin_ws). You can follow the steps in the tutorial.
Read the README in the BKI semantic repository and do the following steps:
1. Build the repository with catkin. Run the following commands in your catkin workspace
(catkin_ws):
• cd src/
• git clone https://github.com/ganlumomo/BKISemanticMapping
• cd ..
• catkin_make
• source catkin_ws/devel/setup.bash
2. Download semantic KITTI dataset sequence 04 data from https://drive.google.com/file/d/19Dv1jQqf-VGKS2qvbygFlUzQoSvu17E5/view and uncompress it into the data folder.
3. Run the demo with the following command:
• roslaunch semantic_bki semantickitti_node.launch

### Simulation

![](https://github.com/DhyeyR-007/Robot-Mapping/blob/main/BKI%20Semantic%20Mapping%20Simulation.gif)
