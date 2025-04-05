% PID tuner

Gs = tf ([0.1026],[0.13677,1])
Gc = pidtune(Gs, 'PI')

pidTuner(Gs,Gc)