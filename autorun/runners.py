import importlib
import sys
import subprocess
import itertools
import os
from operator import mul


def main():
    print sys.argv
    try:
        module_path = sys.argv[1]
        config = importlib.import_module(module_path)
    except IndexError:
        print "Error: Module path not specified"
        return -1
    except ImportError:
        print "Error: This module does not exist"
        return -1
    # Iterating through all programmed tests
    for e in config.execs:
        if e.active:
            print '\033[92m'+"Going to run %s " % e.name + '\033[0m'
            print "This test contains %s executions with %s runs/execution" % \
                (reduce(mul, [len(el[0]) for el in e.variables.values()]), 
                 sum([r.active for r in e.runs]))
            # Creating the needed paths
            for path in e.paths.itervalues():
                try:
                    subprocess.check_call(
                        "./run.sh mkdir -p %s > /dev/null" % path, shell=True)
                except:
                    pass
                print "Created path %s" % path
            for make in e.makes:
                subprocess.check_call(
                    "./run.sh \"cd %s; %s\"" % (make['path'], make['make']), shell=True)
                print "Built software"
            for element in itertools.product(*zip(*e.variables.values())[0]):
                my_env = os.environ.copy()
                env_vars = {
                    key: expr % value for key, value, expr in
                    zip(e.variables.keys(), element, zip(*e.variables.values())[1])
                }
                env_vars.update(e.paths)
                my_env.update(env_vars)
                print "Added environment variables: %s" % " ".join(itertools.chain(*env_vars.items()))
                my_env["PATH"] = "/usr/sbin:/sbin:" + my_env["PATH"]
                for run in e.runs:
                    if run.active:
                        print "Executing %s" % run.name
                        run_vars = list(
                            itertools.chain(*[["--"+p, str(v)] for p, v in run.parameters.iteritems()]))
                        subprocess.call(
                            ["./run.sh"] + [run.command] + run_vars, env=my_env)

if __name__ == '__main__':
    main()
