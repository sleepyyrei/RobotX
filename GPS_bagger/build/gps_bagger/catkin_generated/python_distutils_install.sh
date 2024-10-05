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

echo_and_run cd "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/song/Rei_WS/RobotX/GPS_bagger/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/song/Rei_WS/RobotX/GPS_bagger/install/lib/python3/dist-packages:/home/song/Rei_WS/RobotX/GPS_bagger/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/song/Rei_WS/RobotX/GPS_bagger/build" \
    "/usr/bin/python3" \
    "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/setup.py" \
     \
    build --build-base "/home/song/Rei_WS/RobotX/GPS_bagger/build/gps_bagger" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/song/Rei_WS/RobotX/GPS_bagger/install" --install-scripts="/home/song/Rei_WS/RobotX/GPS_bagger/install/bin"
