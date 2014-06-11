#!/usr/bin/python

import sys
import os
import tempfile
import fileinput
import argparse
import numpy as np


#-----------------------------------------------------------------------------
# Argument parsing: help message

helpmsg = '''Extracts data from FILES, compiles some stats, optionally displays an 
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

#-----------------------------------------------------------------------------
# Argument parsing

parser = argparse.ArgumentParser(description=helpmsg, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("-s", "--skip-first", dest="skip_first", action='store_true',
                  help="skip the first value for each record (useful when there is a costly initialization)")
parser.add_argument("--hist", dest="display_hist", action="store_true",
                  help="display an histogram of retrieved values")
parser.add_argument("--hist-bins", dest="num_hist_bins", default=10, type=int,
                  help="number of bins for the histogram (defaults to 10)")
parser.add_argument("--hist-bound-min", dest="hist_bound_min", default=-float("inf"), type=float,
                  help="for the histogram, ignore values lower than MIN", metavar="MIN")
parser.add_argument("--hist-bound-max", dest="hist_bound_max", default=float("inf"), type=float,
                  help="for the histogram, ignore values greater than MAX", metavar="MAX")
parser.add_argument("-o", "--ofile", dest="outputfilename",
                  help="if reading from stdin, copy stdin to FILE", metavar="FILE")
parser.add_argument('pattern', nargs='?', help="the pattern to search for. If not given then report all variables found.", default=None)
parser.add_argument('field', nargs='?', help="the field number where to get the value from. Defaults to 2.", default=None)
parser.add_argument('files', nargs='*', help="the input files to read data from. Defaults to stdin.", default=None)
args = parser.parse_args()


# Parse the positional arguments
pattern = None
field = None
infiles = []

for a in [args.pattern, args.field]+[args.files]:
  if a is None or len(a)==0:
    continue
  try:
    v = int(a)
    if field is not None:
      sys.exit("bad argument: got " + a + " while I already have %d " % field + "as field value.")
    field = v
    continue
  except:
    pass
  
  if os.path.isfile(a):
    infiles.append(a)
    continue
  else:
    if pattern is not None:
      sys.exit("bad argument: got " + a + " that I interpret as a pattern, but I already have " 
                + pattern + " as pattern, and I only handle a single pattern")
    pattern = a
    continue
    
  sys.exit("bad argument: " + a)

auto_pattern = False
if pattern is None:
  auto_pattern = True
if field is None and not auto_pattern:
  print "You did not provide a field number, trying with 2"
  field = 2
  
  
#-----------------------------------------------------------------------------
# global variables

class DataInfo:
  '''A class to hold records for a variable'''
  def __init__(self):
    # the pattern (i.e. name) for this variable
    self.pattern = ''
    # to display the stats for each variable in order they appeared
    self.i = 0
    # the data itself
    self.records = []
    
# the data we are going to work on
data = {}
# the outputfile, if any
outputfile = None
# counter to record the order in which variables appear
counter = 0



#-----------------------------------------------------------------------------
# Parse the input

for line in fileinput.input(infiles):
  
  # in case we are reading from stdin, we want to copy all the data to a log
  # file so that it can be further analyzed later on.
  # in case no outputfile was given as argument, we create a temp file
  # (this has to be done after reading the first line)
  if fileinput.isstdin():
    if outputfile is None:
      if args.outputfilename is None:
        fd, args.outputfilename = tempfile.mkstemp(prefix='diststat-')
        outputfile = os.fdopen(fd, 'w')
      else:
        outputfile = open(args.outputfilename, 'w')

  
  if args.skip_first and fileinput.isfirstline():
    seen_count = {}
  
  # if we are reading from stdin, then copy to stdout and the outputfile
  if fileinput.isstdin():
    sys.stdout.write(line)
    
  # if we are logging to a file
  if outputfile:
    outputfile.write(line)

  # search for the pattern and get the value
  if not auto_pattern:
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

  # store in database
  if key not in data:
    if args.skip_first:
      if key not in seen_count:
        seen_count[key] = 1
      else:
        seen_count[key] += 1
      if seen_count[key]<=1:
        continue
    
    data[key] = DataInfo()
    data[key].i = counter
    counter += 1
    data[key].pattern = key
  data[key].records.append(val)

#-----------------------------------------------------------------------------
# Process the data

# get rid of variables with only one record
data = [d for d in data.values() if len(d.records)>1]
if len(data)==0:
  sys.exit(0)

if args.skip_first:
  for d in data:
    d.records = d.records[1:]

# sort in order of appearance
data.sort(key=lambda d: d.i)


#-----------------------------------------------------------------------------
# report as a table

# cleanup the patterns
for d in data:
  if d.pattern.endswith(':'):
    d.pattern = d.pattern[:-1]

# create the table: first row is the header
table = [["pattern", "#", "mean", "std", "min", "max"]]
for d in data:
  a = np.asarray(d.records)
  table.append([d.pattern, str(len(d.records))] + map(lambda v: "%.2f" % v, [np.average(a), np.std(a), np.min(a), np.max(a)]))

# if we are logging to a file
if outputfile:
  print ''
  print ''

# print the table nicely
col_widths = [max(len(x) for x in col) for col in zip(*table)]
horline = '-' * (sum(col_widths) + (len(col_widths)-1)*3 + 4)
print horline
for i, line in enumerate(table):
  valstrs = ["{:{}}".format(x, col_widths[c]) for c, x in enumerate(line)]
  print "| " + " | ".join(valstrs) + " |"
  if i==0:
    print horline
print horline


# if we are logging to a file
if outputfile:
  print ''
  print "stdin was saved to", args.outputfilename

  
#-----------------------------------------------------------------------------
# histogram

if args.display_hist:
  from matplotlib import pyplot as plt
  for d in data:
    plt.figure(d.pattern)
    X = [x for x in d.records if x>args.hist_bound_min and x<args.hist_bound_max]
    plt.hist(X, bins=args.num_hist_bins)
  plt.show()
