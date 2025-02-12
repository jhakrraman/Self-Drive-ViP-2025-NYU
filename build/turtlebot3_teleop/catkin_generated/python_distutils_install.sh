#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/linux/catkin_s25/src/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/linux/catkin_s25/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/linux/catkin_s25/install/lib/python3/dist-packages:/home/linux/catkin_s25/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/linux/catkin_s25/build" \
    "/usr/bin/python3" \
    "/home/linux/catkin_s25/src/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/linux/catkin_s25/build/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/linux/catkin_s25/install" --install-scripts="/home/linux/catkin_s25/install/bin"
