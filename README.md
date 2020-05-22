# ROS GUI Server

The ROS GUI Server package is the component of the ROS GUI system which runs on the target robot. It includes a ros_gui_server_node which subscribes to an nav_msgs/Odometry topic, exposes Odometry, rviz visualization, and robot navigation through actionlib actions to the ROS GUI Client.


## Configuration

Setting up the ros_gui_server package in your ROS environment can get a bit involved, however I intented to give thorough enough explanations so that anyone familiar with ROS, or Linux in general could set this up easily.

The first few step are fairly standard, simply clone this repository in a catkin workspace on the host device (robot), run catkin_make to generate a new setup.bash, source it from the devel folder, and execute the command `rosdep install ros_gui_server` to gather all of the dependancies.

#### URDF Server Setup 

In order a to serve the URDF of your robot to the GUI in the browser, its files need to be served via the http protocol. To accomplish this, we can set up an Apache2 webserver on the host device and make some modifications to it.

First, we need to download Apache to the host device:

```
sudo apt update
sudo apt install apache2
```

Then, we'll need to enable header modification:

```
sudo a2enmod headers
```

Next, we'll create a config for our new site, we can just copy the default one:

```
cd /etc/apache2/sites-available/
sudo cp 000-default.conf gui.conf
```

Then, we need to add our header modification mentioned previously, start by opening our newly created config file:

```
sudo nano gui.conf
```

Scroll down to near the end of the <VirtualHost> tag and add the following line into it:

```
Header set Access-Control-Allow-Origin *
```

The above line allows the URDF to be served to the client's browser without causing any security errors, after saving the gui.conf file, we'll need to disable the default site, enable our new one, and restart the apache server.

```
sudo a2dissite 000-default.conf
sudo a2ensite gui.conf
sudo service apache2 restart
```

Now that our apache server is properly configured, we need to copy the URDF (typically in a robot_description package) into the content directory for the site. To do this simply execute these commands, replacing <robot-description> with the path to your robot_description package.
  
```
sudo rm /var/www/html/index.html
sudo mkdir /var/www/html/urdf
sudo cp -r <robot_description>/ /var/www/html/urdf/
```

Then, we can reload the apache server to serve the new content, and our apache server config is complete!

```
sudo service apache2 reload
```
