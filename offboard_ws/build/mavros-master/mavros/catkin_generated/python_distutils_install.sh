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

echo_and_run cd "/home/xhe/offboard_ws/src/mavros-master/mavros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/xhe/offboard_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/xhe/offboard_ws/install/lib/python2.7/dist-packages:/home/xhe/offboard_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/xhe/offboard_ws/build" \
    "/usr/bin/python2" \
    "/home/xhe/offboard_ws/src/mavros-master/mavros/setup.py" \
     \
    build --build-base "/home/xhe/offboard_ws/build/mavros-master/mavros" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/xhe/offboard_ws/install" --install-scripts="/home/xhe/offboard_ws/install/bin"
