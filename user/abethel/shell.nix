{ pkgs ? import <nixpkgs> { } }:
with pkgs;

let
  ros_progs = [
    # This was pulled from `ls /opt/ros/noetic/bin`.
    "catkin_find"
    "catkin_init_workspace"
    "catkin_make"
    "catkin_make_isolated"
    "catkin_test_results"
    "catkin_topological_order"
    "ompl_benchmark_statistics.py"
    "rosbag"
    "rosboost-cfg"
    "rosclean"
    "rosconsole"
    "roscore"
    "roscreate-pkg"
    "rosgraph"
    "roslaunch"
    "roslaunch-complete"
    "roslaunch-deps"
    "roslaunch-logs"
    "rosmake"
    "rosmaster"
    "rosmsg"
    "rosmsg-proto"
    "rosnode"
    "rospack"
    "rosparam"
    "rosrun"
    "rosservice"
    "rossrv"
    "rosstack"
    "rostest"
    "rostopic"
    "rosunit"
    "roswtf"
    "rqt"
    "rviz"
    "tf_remap"
    "view_frames"

    "catkin_create_pkg"
  ];
  entrypoint = writeScript "entrypoint" ''
    #!/bin/bash
    set -e

    source /opt/ros/$ROS_DISTRO/setup.bash --
    source /home/alex/src/lunabotics/ros/catkin_ws/devel/setup.bash --

    exec "$@"
  '';
in
mkShell {
  nativeBuildInputs = [
    # xhost is in my .bashrc, not necessary here.
    (writeScriptBin "xhost" ''
      #!/bin/sh
      # do nothing
    '')
    python310

    # Make wrappers for the ROS programs.
    (symlinkJoin {
      name = "ros-programs";
      paths = map
        (
          # conman is my own container manager; I'll probably publish
          # it at some point.
          prog_name: writeScriptBin prog_name ''
            #!${bash}/bin/bash
            conman run -- ${./ros-env} ${entrypoint} ${prog_name} "$@"
          ''
        )
        ros_progs;
    })

    # Add special program `rosterm` that gives a shell with ROS
    # installed.
    (writeScriptBin "rosterm"
      ''
        #!${bash}/bin/bash
        conman run -- ${./ros-env} ${entrypoint} bash "$@"
      '')

    (import ./cantools { inherit pkgs; })
    (import ./arduino-nix { inherit pkgs; })
    ncurses.dev
    can-utils
    # (import ./libcan { inherit pkgs; })
  ];
}
