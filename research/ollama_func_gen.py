import re
from datetime import datetime, timedelta
import sys
import os

import pandas as pd
import ollama

# Check if model parameter is provided
if len(sys.argv) < 2:
    print("Usage: python object_size_generate.py model_name")
    sys.exit(1)

# Get model name from command line argument
model = sys.argv[1]

client = ollama.Client(host='http://localhost:11434')

mdl = model.replace(':', '_')
os.makedirs(mdl, exist_ok=True)
os.chdir(f"./{mdl}")

f_name = "object_height"

loop_time = []
for k in range(1, 31):
    i = 1
    df = pd.DataFrame()
    start_dt = datetime.now()
    while True:
        begin_dt = datetime.now()
        response = ollama.generate(
            model=model,
            prompt=f"""Create python function {f_name} which calculates object height
            by known image height in pixels
            Input parameters:
            * f - focal length in mm, default 35 mm
            * h - image height in pixels, default 1152
            * p - pixel size in um, default 3.45 um
            * d - object distance in meters, default 6.5 m
            Returns object height in mm as absolute float value
            Raises RuntimeError if input parameters are invalid (negative or equal to zero)
            """,
        )
        out_str = response.response
        print(f"Response:\n{out_str}")
        match = re.search(r"(?:`{3}\w*|\[PYTHON\])\n(.+?)(?:`{3}|\[\/PYTHON\])", out_str, flags=re.DOTALL)
        ans = pd.Series(index=["code", "res", "time"], dtype=object)
        if match:
            ans["code"] = match.group(1)
        elif "```" not in out_str and "PYTHON" not in out_str:
            ans["code"] = out_str
        else:
            print('Code parse error: can`t parse')
            continue

        # Create namespace dictionary
        namespace = {}

        try:
            # Execute the string in the namespace
            exec(ans["code"], namespace)

            # Get the function from namespace
            my_function = namespace[f_name]

            print(
                "Function output: ", round(my_function(), 1)
            )  # Expect output: 738,1 mm

            # === CHECKS ===
            err_check = False
            try:
                my_function(f=-35, h=1152, p=3.45, d=6.5)
            except RuntimeError:
                err_check = True
            else:
                print("Error check isn`t passed")

            if (
                err_check
                and round(my_function(), 0) == 738
                and round(my_function(f=35, h=1152, p=3.45, d=6.5), 0) == 738
                and round(my_function(f=35, h=1152, p=3.45, d=5.9), 0) == 670
            ):
                ans["res"] = "OK"
            else:
                ans["res"] = "WRONG"

        except Exception as e:
            print(f"Error: {e}")
            ans["res"] = "WRONG"

        finally:
            ans["time"] = datetime.now() - begin_dt
            t: timedelta = ans["time"]
            print(k, i, ans["res"], "time (s):", t.seconds, mdl, sep=', ')
            df = pd.concat([df, ans.to_frame().T], axis=0, ignore_index=True)
            i += 1
            if ans["res"] == "OK":
                break

    df.to_csv(f"answers_{k}.csv")
    with open(f"code_{k}.txt", mode="w", encoding="utf-8") as code:
        txt: str = ans["code"]
        txt = txt.encode(errors="ignore").decode(errors="ignore")
        code.write(ans["code"])

    loop_time.append(f"{datetime.now() - start_dt}")

    with open("loop_times", mode="w") as loop_times:
        for time in loop_time:
            loop_times.write(f"{time}\n")
