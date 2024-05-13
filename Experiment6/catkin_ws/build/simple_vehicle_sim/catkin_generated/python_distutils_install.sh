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

echo_and_run cd "/home/sa/RE510_2024/Experiment6/catkin_ws/src/simple_vehicle_sim"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sa/RE510_2024/Experiment6/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sa/RE510_2024/Experiment6/catkin_ws/install/lib/python3/dist-packages:/home/sa/RE510_2024/Experiment6/catkin_ws/build/simple_vehicle_sim/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sa/RE510_2024/Experiment6/catkin_ws/build/simple_vehicle_sim" \
    "/home/sa/anaconda3/envs/re510_exp6/bin/python3" \
    "/home/sa/RE510_2024/Experiment6/catkin_ws/src/simple_vehicle_sim/setup.py" \
     \
    build --build-base "/home/sa/RE510_2024/Experiment6/catkin_ws/build/simple_vehicle_sim" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sa/RE510_2024/Experiment6/catkin_ws/install" --install-scripts="/home/sa/RE510_2024/Experiment6/catkin_ws/install/bin"
