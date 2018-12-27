# Chimney RViz Plugins

chimney-rviz-plugins is a stack for the visualization packages which you can easily
edit some generic data in RViz GUI.

## Gallery

### [Editable Polygon](nodes/editable_polygon/README.md)
![image](.readme/editable-polygon.gif)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

ROS kinetic (Ubuntu 16.04) or ROS melodic (Ubuntu 18.04)

A detailed ROS install tutorial can be founded [here](http://www.ros.org/install/).

### Installing

A step by step series of examples that tell you how to get a development env running

Firstly, clone the project to your local machine

```
mkdir -p /usr/local/src/chimney-rviz-plugins/src && cd /usr/local/src/chimney-rviz-plugins/src && catkin_init_workspace
git clone https://github.com/smallchimney/chimney-rviz-plugins.git
```

Then compile it with catkin

```
cd .. && catkin_make
```

Now just add the plugins context to your machine

```
echo "source /usr/local/src/chimney-rviz-plugins/devel/setup.bash" >> $HOME/.bashrc
```

Or if you are using zsh

```
echo "source /usr/local/src/chimney-rviz-plugins/devel/setup.zsh" >> $HOME/.zshrc
```

Open an new terminator and open RViz, now you can see chimney-rviz-plugins has been add into RViz.

For more information, see the detailed RViz tutorial [here](http://wiki.ros.org/rviz).

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/smallchimney/chimney-rviz-plugins/tags). 

## Authors

* **Chimney Xu** - *Initial work* - [SmallChimney](https://github.com/SmallChimney)

See also the list of [contributors](https://github.com/smallchimney/chimney-rviz-plugins/contributors) who participated in this project.

## License

This project is licensed under the Apache License V2 - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
