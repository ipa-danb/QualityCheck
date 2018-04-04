#!/usr/bin/env python

from __future__ import print_function  # Only needed for Python 2
import time
import string
import sys
import commands
import argparse

def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")

def get_cpumem(pid):
    d = [i for i in commands.getoutput("ps aux").split("\n")
        if i.split()[1] == str(pid)]
    return (float(d[0].split()[2]), float(d[0].split()[3])) if d else None

def get_pid(name):
    d = [i for i in commands.getoutput("ps aux | grep {}".format(name)).split("\n")
        if i.split()[11].find(name) >= 0]
    choice = query_yes_no("Please verify the correct process:\n{}".format(d[0]))
    if choice:
        print("Using PID: {}".format(d[0].split()[1]))
        return d[0].split()[1]
    else:
        exit(1)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("id", help="The PID or NAME of the wanted process")
    parser.add_argument('-f', '--file')
    args = parser.parse_args()

    pid = args.id
    if not all(i in string.digits for i in pid):
        print("Trying to find PID of '%s'" % pid)
        pid = get_pid(pid)

    
    if args.file: 
        print("Logging to file: {}".format(args.file))
        f = open(args.file, 'a')
    else:
        f=sys.stdout

    print("%CPU\t%MEM", file=f)
    try:
        while True:
            x = get_cpumem(pid)
            if not x:
                print("no such process")
                exit(1)
            print("%.2f\t%.2f" % x, file=f)
            time.sleep(0.5)
    except KeyboardInterrupt:
        if args.file:
            f.close()
        print
        exit(0)
