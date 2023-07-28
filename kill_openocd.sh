pid=`sudo ss -tlnp | grep 6666 | awk -F 'pid=' '{print $2}' | awk -F ',' '{printf $1}'`

if [ $pid == '']; then
    echo "Not found pid"
    exit 0
else
    echo $pid
fi

kill -9 $pid