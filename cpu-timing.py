import subprocess
import csv
import re

DATASRC = 'data/'
DATAFILES = ['v10-e20.graph', 'v10k-e25k.graph', 'v10k-e50k.graph', 'v10k-e100k.graph']
MAXTHREADS = 16

DEVICE = 'kierkegaard'

fout = csv.writer(open('data/timing.csv', 'w'))
fout.writerow(['version', 'device', 'dataset', 'time'])

for fname in DATAFILES:
    print(fname)
    for num_threads in range(1, MAXTHREADS+1):
        args = ['./build/bin/sssp_cpu_only', \
                '--input='+DATASRC+fname, \
                '--source=0', \
                '--workers='+str(num_threads)]
        print(args)
        output = subprocess.check_output(args)
        last_line = output.split('\n')[-2]
        time_elapsed = re.match(r'Elapsed runtime: (\d+)ms', last_line).group(1)
        print(time_elapsed, 'ms')

        fout.writerow(['CPUx%02d' % (num_threads,), DEVICE, fname, time_elapsed]) 
