```shell
javac KalmanFilter.java 
java KalmanFilter gps.txt gps_smooth.txt 128.0
```

```gnuplot
plot 'gps.txt' using 3:4
plot 'gps_smooth.txt' using 3:4
```

