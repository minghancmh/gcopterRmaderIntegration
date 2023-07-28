<h2> Overview </h2>

This repository is a research based project to try and transform a constrained optimization path planning problem into an unconstrained one. It utilizes both the RMADER and GCOPTER planning algorithms. At present, the RMADER and GCOPTER performance is studied based on varying levels of discretization and CPU available resources. 

[RMADER](https://github.com/mit-acl/rmader) 
<p> </p>

[GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)

At present, this repository has benchmarked the GCOPTER and RMADER optimization times against each other.

<h2> Setup </h2>

<h3> Ubuntu </h3>

Please install [Ubuntu 20.04](https://www.cyberithub.com/how-to-install-ubuntu-20-04-lts-on-windows-10-wsl/), using WSL2.

<h3> ROS </h3>

Please install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).
[Easy Install](http://wiki.ros.org/ROS/Installation/TwoLineInstall/)



<h3> Rviz </h3>

```bash
sudo apt-get install ros-noetic-rviz
```


<h3> Package Setup </h3>

Simply run the commands:

```bash
cd ~/ && mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/minghancmh/gcopterRmaderIntegration.git
sudo apt-get install python3-catkin_tools
cd ..
bash install_nlopt.sh
bash install_and_compile.sh    
```

The script install_and_compile.sh will install [CGAL v4.12.4](https://www.cgal.org/), [GLPK](https://www.gnu.org/software/glpk/) and other ROS packages (check the script for details). It will also compile the repo. This bash script assumes that you already have ROS installed in your machine. 


<h3> Errors </h3>

<h5> package not found or no such package</h5>

Remember to run this script before running any of the roslaunch files. Note that you should be in the root directory of the cloned repository. (ie cd gcopterRmaderIntegration && ls) should show the devel folder.

```bash
source devel/setup.bash
```

<h5> ssl handshake denied </h5>

```bash
pip install --trusted-host pypi.org --trusted-host files.pythonhosted.org <package_name>
```

<h5> check certificate fail </h5>

```bash
git config --global http.sshverify false
```


<h3> Running Simulations </h3>

For gcopter simulation

```bash
roslaunch gcopter multi_obstacle_gcopter.launch
```

For rmader simulation

```bash
roslaunch test_nlopt.launch
```

<h3> Results and Analysis </h3>

The GCOPTER and RMADER algorithms were ran over 50 simulations on identical maps. The seed value was set to "123" for all simulations. The time taken for the algorithms to generate a complete path was then recorded and analysed in the "benchmarking" folder. Results and analysis of this results are shown in the presentation deck in the "benchmarking" folder.

The "benchmarkingLogs" contain all raw logs before parsing and processing.

The logs are parsed using the GcopterFormatter.py and RmaderFormatter.py files.


<h3> For use of VSC in Ubuntu Environment </h3>

Enable the remote desktop VSC extension. Ensure that ssh is running before attempting to connect to ubuntu vm ip addr.

To check status of ssh
```bash
sudo service ssh status
```

To start ssh server
```bash
sudo service ssh start
```

To restart ssh server
```bash
sudo service ssh restart
```

