
import sys
import re

filename = sys.argv[1]

with open(filename) as inpf:
    for line in inpf:
        line = line.strip()
        is_error = re.match("Err1:(.*)$", line)
        if is_error:
            the_bytes = is_error.group(1)
            code = the_bytes.split(" ")
            decoded = ""
            for c in code:
                decoded = decoded + chr(int(c, 16))
            print(decoded)
