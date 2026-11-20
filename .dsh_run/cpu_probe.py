import os, sys, time, glob
pid = sys.argv[1]
def total():
    t = 0
    for f in glob.glob(f"/proc/{pid}/task/*/stat"):
        try:
            parts = open(f).read().rsplit(")", 1)[1].split()
        except Exception:
            continue
        t += int(parts[12]) + int(parts[13])   # utime stime (fields after comm/state)
    return t
prev = total(); prev_t = time.time()
for i in range(int(sys.argv[2])):
    time.sleep(1)
    if not os.path.exists(f"/proc/{pid}"):
        print("[probe] 进程已退出"); break
    cur = total(); now = time.time()
    cpu = (cur - prev) / os.sysconf("SC_CLK_TCK") / (now - prev_t) * 100
    states = sorted({open(f).read().rsplit(")",1)[1].split()[0] for f in glob.glob(f"/proc/{pid}/task/*/stat")})
    nthreads = len(glob.glob(f"/proc/{pid}/task/*/stat"))
    print(f"[{i+1}s] cpu={cpu:6.1f}%  线程数={nthreads}  状态={''.join(states)}", flush=True)
    prev, prev_t = cur, now
