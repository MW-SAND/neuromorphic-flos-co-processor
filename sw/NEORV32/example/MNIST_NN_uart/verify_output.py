import sys

out_filename = "../../../sim/neorv32.uart0.sim_mode.text.out"
verify_filename = "verification.out"

if len(sys.argv) > 1:
    out_filename = sys.argv[1]
if len(sys.argv) > 2:
    verify_filename = sys.argv[2]

print(f"checking out file: {out_filename} with verification file: {verify_filename}")

verify_file = open(verify_filename)
out_file = open(out_filename)

for line in out_file:
    if line.startswith("["):
        veryline = verify_file.readline()
        print(line, end="")
        print(veryline, end="")
        if line == veryline:
            print("verification OK")
            exit()

print("verification failed")
exit(1)
