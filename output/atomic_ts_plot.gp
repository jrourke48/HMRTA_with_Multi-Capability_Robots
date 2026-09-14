set terminal pngcairo size 1200,1000
set output 'atomic_ts.png'
set title 'Atomic Transition System'
set size ratio -1
set grid
set key off
set xrange [-12:60]
set yrange [-12:60]
plot \
    'output/atomic_ts_edges.dat' with lines smooth bezier lc rgb '#999999' lw 2, \
    'output/atomic_ts_nodes.dat' with points pt 7 ps 10.0 lc rgb '#000000', \
    'output/atomic_ts_nodes.dat.labels' with labels tc rgb '#FFFFFF' font ',7' offset 0,0, \
    'output/atomic_ts_edges.dat.labels' with labels tc rgb '#7A0019' font ',5' offset 0,0
