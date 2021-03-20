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

echo_and_run cd "/home/saksham/iris_quad/src/navigation/base_local_planner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/saksham/iris_quad/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/saksham/iris_quad/install/lib/python2.7/dist-packages:/home/saksham/iris_quad/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/saksham/iris_quad/build" \
    "/usr/bin/python2" \
    "/home/saksham/iris_quad/src/navigation/base_local_planner/setup.py" \
     \
    build --build-base "/home/saksham/iris_quad/build/navigation/base_local_planner" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/saksham/iris_quad/install" --install-scripts="/home/saksham/iris_quad/install/bin"
