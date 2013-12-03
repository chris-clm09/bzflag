./bin/bzrflag --window-size=800x800 --default-posnoise=5 --world=maps/kalman.bzw --red-port=50100 --green-port=50101 --red-tanks=1 --green-tanks=1$@ &
sleep 2
python bzagents/duck.py localhost 50101 &
python bzagents/hunter.py localhost 50100 &
