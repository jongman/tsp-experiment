#!/usr/bin/python
# -*- coding: utf-8 -*-

import argparse, subprocess, glob, os, time, json

def make_output_directory(args):
    try:
        os.makedirs("outputs/%s" % args.algorithm.lower())
    except OSError:
        pass

def get_output_path(args, input_file):
    return "outputs/%s/%s" % (args.algorithm.lower(),
            os.path.basename(input_file).replace("inp", "outp"))

def run_with_time_limit(command_line, time_limit):
    start = time.time()
    process = subprocess.Popen(command_line, shell=True)
    while True:
        now = time.time()
        if process.poll() != None or now > start + time_limit: break
        time.sleep(0.01)
    if process.returncode == None:
        process.kill()
        return None
    return now - start

def test_with(args):
    make_output_directory(args)
    log_entry = {"algorithm": args.algorithm, "times": {}}
    input_files = list(sorted(glob.glob("inputs/*.txt")))
    if args.n != None:
        input_files = input_files[:int(args.n)]
    for input_file in input_files:
        output_file = get_output_path(args, input_file)
        runtime = run_with_time_limit(" ".join([args.executable,
            args.algorithm,
            input_file, output_file]),
            args.time_limit)
        if runtime == None:
            runtime = "TLE"
            print "Time limit exceeded for %s." % input_file
        else:
            res = open(output_file).read().strip()
            print "Solved %s within %g seconds." % (input_file, runtime)
        log_entry["times"][input_file] = runtime
        if runtime == "TLE": break
    return log_entry

def leave_log(entry):
    fp = open("tests.log", "a")
    fp.write("%s\n" % json.dumps(entry))
    fp.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("executable")
    parser.add_argument("algorithm")
    parser.add_argument("-t", "--time-limit", default=10.0, type=float)
    parser.add_argument("-n", default=None)
    args = parser.parse_args()

    log_entry = test_with(args)
    leave_log(log_entry)

if __name__ == "__main__":
    main()
