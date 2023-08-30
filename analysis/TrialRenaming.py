import glob
import os

trial_num = 9
dir = "..\\results_Trial" + str(trial_num + 1) + "\\"

file_base = "StationaryTest10mW-*,*-#0"
files_sca = glob.glob(dir + file_base + ".sca")
files_vec = glob.glob(dir + file_base + ".vec")

for file in files_sca:
    fname_new = file[:-5] + str(trial_num) + ".sca"
    print(fname_new)
    os.rename(file, fname_new)

for file in files_vec:
    fname_new = file[:-5] + str(trial_num) + ".vec"
    print(fname_new)
    os.rename(file, fname_new)
    