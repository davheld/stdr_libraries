#!/usr/bin/python

import sys
import os
import tempfile
import fileinput
import argparse
import numpy as np

# the data we are going to work on
data = {}
outputfile = None

description = '''Extracts data from FILES, compiles some stats, optionally displays an 
histogram.

In autonomous mode, each line that matches the following pattern:
  description [a number]
is gathered and statistics are extracted across the several occurences of the
same description

In non autonomous mode, options specify the pattern and field number to search
for when extracting data. This is similar to awk:
$> aProgramToAnalyze | %prog pattern 2
will extract the same data as
$> aProgramToAnalyze | awk '/pattern/ {print $2}'

If FILES is not given, data is read from stdin. In this case, stdin is copied
to stdout, and the reported stats are added at the end. Also, the input will be 
saved to a file, so that future stats can be extracted later.

Examples:
distribution_stats pattern 2
  will read data from stdin and search for lines with the pattern, and report
  stats about the 2nd field in the line
  
distribution_stats pattern 2 file1 file2
  will read data from file1 and file2

distribution_stats -a
  will register a pattern for each line, matching patterns across repetitions and
  giving stats about the second field:
  e.g. for the following input:
    desc_a 1
    desc_b 10
    desc_a 2
    desc_b 12
  it will report that desc_a appeared 2 times with a mean of 1.5, and that desc_b
  appeared 2 times with a mean of 11
'''

parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--hist", dest="display_hist", action="store_true",
                  help="display an histogram of retrieved values")
parser.add_argument("--bins", dest="num_hist_bins", default=10, type=int)
parser.add_argument("-o", "--ofile", dest="outputfilename",
                  help="copy stdin to FILE", metavar="FILE")
parser.add_argument("-a", "--auto", help="turn on the automatic mode", action='store_true')
parser.add_argument('remainder', nargs=argparse.REMAINDER)
args = parser.parse_args()


if not args.auto:
  if len(args.remainder)<2:
    print 'Error: pattern and field number missing (non automatic mode)'
    sys.exit(1)
  pattern = args.remainder[0]
  field = int(args.remainder[1])
  infiles = args.remainder[2:]
else:
  infiles = args.remainder


if args.display_hist:
  from matplotlib import pyplot as plt
  

class DataInfo:
  def __init__(self):
    self.pattern = ''
    self.i = 0
    self.data = []
    
counter = 0

for line in fileinput.input(infiles):
  
  if fileinput.isstdin():
    if outputfile is None:
      if args.outputfilename is None:
        fd, args.outputfilename = tempfile.mkstemp(prefix='diststat-')
        outputfile = os.fdopen(fd, 'w')
      else:
        outputfile = open(args.outputfilename, 'w')
    sys.stdout.write(line)
    outputfile.write(line)
  
  if not args.auto:
    if pattern not in line:
      continue
    key = pattern
    val = float(line.split()[int(field)-1])
  else:
    tokens = line.split()
    if len(tokens)<2:
      continue
    key = tokens[0]
    try:
      val = float(tokens[1])
    except:
      continue
  
  if key not in data:
    d = DataInfo()
    d.i = counter
    counter = counter + 1
    d.data.append(val)
    d.pattern = key
    data[key] = d
  else:
    data[key].data.append(val)


D = sorted([d for d in data.values() if len(d.data)>1], key=lambda v: v.i)
if len(D)==0:
  sys.exit(0)

maxlen = 0
for d in D:
  if d.pattern.endswith(':'):
    d.pattern = d.pattern[:-1]
  if len(d.pattern)>maxlen:
    maxlen = len(d.pattern)


def print_table(table):
  col_width = [max(len(x) for x in col) for col in zip(*table)]
  horline = '-' * (sum(col_width) + (len(col_width)-1)*3 + 4)
  print horline
  for i, line in enumerate(table):
    print "| " + " | ".join("{:{}}".format(x, col_width[i])
                                for i, x in enumerate(line)) + " |"
    if i==0:
      print horline
  print horline

table = [["pattern", "#", "mean", "std", "min", "max"]]
for d in D:
  a = np.asarray(d.data)
  table.append([d.pattern, str(len(d.data))] + map(lambda v: "%.2f" % v, [np.average(a), np.std(a), np.min(a), np.max(a)]))

if outputfile:
  print ''
  print ''
print_table(table)


if outputfile:
  print ''
  print "stdin was saved to", args.outputfilename

if args.display_hist:
  for d in D:
    plt.figure(d.pattern)
    plt.hist(d.data, bins=args.num_hist_bins)
  plt.show()
