./bin/bzrflag --world=maps/four_ls.bzw --red-port=50100 --green-port=50101 --purple-port=50102 --blue-port=50103 $@ &
sleep 5
./better.py localhost 50100 &
./better.py localhost 50101 &
./better.py localhost 50102 &
./better.py localhost 50103 &

