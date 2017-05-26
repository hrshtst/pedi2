./demo.sh
./joystick_ctrl ../model/mighty.zkc -env ../model/env/random_room.zkc -R cmd.log
rm -rf operator_view
mkdir -p operator_view
mv *.png operator_view
python3 make_video.py operator_view avi 100
mv video.mp4 operator_view.mp4
rk_anim ../model/mighty.zkc motion.zvs -env ../model/env/random_room.zkc -x 10.5 -y -- 2 -z 16 -pan -- 0 -tilt -- -60 -capture png
rm -rf bird_view
mkdir -p bird_view
mv *.png bird_view
python3 make_video.py bird_view avi 30
mv video.mp4 bird_view.mp4

