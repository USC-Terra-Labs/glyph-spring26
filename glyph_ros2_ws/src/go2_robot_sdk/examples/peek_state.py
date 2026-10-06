#!/usr/bin/env python3
import subprocess, shlex, time

TOPICS = [
    "/lowstate",
    "/multiplestate",
    "/sportmodestate",
    "/lf/lowstate",
    "/lf/sportmodestate",
    "/rtc/state",
    "/api/robot_state/response"
]

def echo_once(topic):
    cmd = f"ros2 topic echo {topic} --once"
    try:
        print("===", topic, "===")
        out = subprocess.check_output(shlex.split(cmd), stderr=subprocess.STDOUT, timeout=8)
        print(out.decode(errors="ignore"))
    except subprocess.CalledProcessError as e:
        print("Command failed:", e.returncode)
        print(e.output.decode(errors="ignore"))
    except subprocess.TimeoutExpired:
        print("Timed out (no message published in window)")
    except Exception as ex:
        print("Error:", ex)

if __name__ == '__main__':
    for t in TOPICS:
        echo_once(t)
        time.sleep(0.2)
