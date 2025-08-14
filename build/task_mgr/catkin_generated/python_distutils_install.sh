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

echo_and_run cd "/home/agilex/AgileXDemo/src/task_mgr"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/agilex/AgileXDemo/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/agilex/AgileXDemo/install/lib/python3/dist-packages:/home/agilex/AgileXDemo/build/task_mgr/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/agilex/AgileXDemo/build/task_mgr" \
    "/home/agilex/miniconda3/bin/python3" \
    "/home/agilex/AgileXDemo/src/task_mgr/setup.py" \
    egg_info --egg-base /home/agilex/AgileXDemo/build/task_mgr \
    build --build-base "/home/agilex/AgileXDemo/build/task_mgr" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/agilex/AgileXDemo/install" --install-scripts="/home/agilex/AgileXDemo/install/bin"
