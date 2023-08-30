import sqlite3
import glob
import numpy
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import math

file_loc = "..\\results\\"

config_name = "StationaryTest10mW"

step = 77.5
min_x = 75
max_x = 695.0
min_y = 410
max_y = 1030.0

iterations = 1

excludes = [[75.0, 230.0, 385.0, 540.0, 695.0],
            [410.0, 565.0, 720.0, 875.0, 1030.0]]

print("X: %2d, Y: %2d" % ((max_y - min_y) / step, (max_x - min_x) / step))

map = numpy.zeros([int((max_y - min_y) / step) + 1, int((max_x - min_x) / step) + 1], dtype=float)

scalars = glob.glob(file_loc + config_name + '-*,*-#0.sca')

map_linear2 = numpy.ndarray((3,0))
xs = numpy.array([])
ys = numpy.array([])
tps = numpy.array([])
sdevs = numpy.array([])

idx = 0

for scalar_file in scalars:
    sca = sqlite3.connect(scalar_file)
    vec = sqlite3.connect(scalar_file[:-3] + "vec")

    sca_cur = sca.cursor()
    x_pos_str = sca_cur.execute("SELECT paramValue FROM parameter WHERE moduleName = ? AND paramName = ?",('Multihop.hostR.mobility','initialX')).fetchall()[0][0][:-1]
    x_pos = float(x_pos_str)
    y_pos_str = sca_cur.execute("SELECT paramValue FROM parameter WHERE moduleName = ? AND paramName = ?",('Multihop.hostR.mobility','initialY')).fetchall()[0][0][:-1]
    y_pos = float(y_pos_str)
    
    if x_pos not in excludes[0] or y_pos not in excludes[1]:
        fname_base = scalar_file[:-5]
        thpts_file = numpy.array([])
        for i in range(0, iterations):

            fname_vec = fname_base + str(i) + ".vec"

            vec_cur = sqlite3.connect(fname_vec).cursor()
            id = vec_cur.execute("SELECT vectorId FROM vector WHERE moduleName = ? AND vectorName = ?", ('Multihop.hostA.app[0]', 'throughput:vector')).fetchall()
            # thpts = vec_cur.execute("SELECT vectorCount, vectorSum FROM vector WHERE moduleName = ? AND vectorName = ?", ('Multihop.hostA.app[0]', 'throughput:vector')).fetchall()
            if (len(id) == 0):
                # throughput = 0
                thpts_file = numpy.append(thpts_file, 0)
            else:
                thpts_str = vec_cur.execute("SELECT value FROM vectorData WHERE vectorId = ? AND simtimeRaw > 2000000000000", (id[0])).fetchall()
                thpts = [float(value[0]) for value in thpts_str]
                thpts_file = numpy.append(thpts_file, sum(thpts) / len(thpts))
                # thpt_cnt_str, thpt_sum_str = thpts[0]
                # thpt_cnt = int(thpt_cnt_str)
                # thpt_sum = float(thpt_sum_str)
                # throughput = thpt_sum / thpt_cnt

        throughput = sum(thpts_file) / len(thpts_file)
        mean_diff_sqrd = [pow(val - throughput, 2) for val in thpts_file]
        std_dev = math.sqrt(sum(mean_diff_sqrd) / len(mean_diff_sqrd))
        print("%5.3f, dev=%5.3f bps at (%4.1f, %4.1f)" % (throughput, std_dev, x_pos, y_pos))
        map[int((y_pos - min_y) / step)][int((x_pos - min_x) / step)] = throughput
        xs = numpy.append(xs, x_pos)
        ys = numpy.append(ys, y_pos)
        tps = numpy.append(tps, throughput)
        sdevs = numpy.append(sdevs, std_dev)

#print(map_linear2)

x_len = len(map[0])
y_len = len(map)

x_vals = numpy.zeros(x_len)
y_vals = numpy.zeros(y_len)

map_linear = numpy.zeros((3, x_len * y_len))

i = 0
for x in range(0, x_len):
    for y in range(0, y_len):
        map_linear[0][i] = (step * x) + min_x
        map_linear[1][i] = (step * y) + min_y
        map_linear[2][i] = map[y][x]
        i += 1

#map_linear[0] = map_linear[0][::-1]
sc = plt.scatter(xs, ys, s=100, c=tps, cmap="plasma")
#sc = plt.scatter(map_linear[0], map_linear[1], s=100, c=map_linear[2], cmap="plasma")
plt.colorbar(sc, label="Throughput (bits per second)")
# for i in range(0, len(sdevs)):
#     txt = plt.text(xs[i],ys[i] + 20, "σ: {:2.2f}".format(sdevs[i]), fontsize=7, horizontalalignment='center', verticalalignment='center')
#     txt.set_path_effects([pe.withStroke(linewidth=1, foreground='w')])
txt1 = plt.text(152.5,487.5, "Receiver", horizontalalignment='center', verticalalignment='center')
txt2 = plt.text(307.5,797.5, "Node", horizontalalignment='center', verticalalignment='center')
txt3 = plt.text(617.5,952.5, "Node", horizontalalignment='center', verticalalignment='center')
txt1.set_path_effects([pe.withStroke(linewidth=2, foreground='w')])
txt2.set_path_effects([pe.withStroke(linewidth=2, foreground='w')])
txt3.set_path_effects([pe.withStroke(linewidth=2, foreground='w')])
plt.title("Throughput vs Location")
plt.xlabel("X Location (meters)")
plt.ylabel("Y Location (meters)")
y_lims = plt.ylim()
plt.ylim(y_lims[1], y_lims[0])
plt.show()