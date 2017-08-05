if git rev-parse --is-inside-work-tree &> /dev/null; then
    git checkout demo
    # git checkout demo.20161013
fi
make
./joystick_ctrl ../model/mighty.zkc -env ../model/env/random_room.zkc -width 1000 -height 1000
