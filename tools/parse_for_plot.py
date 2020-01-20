import sys
import string
import math
from matplotlib import pyplot as plt
import numpy as nm
from datetime import datetime
import array

auto_time=[];
auto_time_raw=[];  # offset time;
create_time=[];
create_time.append(0);
auto_x=[]; # X pose in each step;
auto_y=[];
auto_h=[];
start_time=datetime.now();
end_time=start_time;
init_time=start_time;
last_time=start_time;
data=[];
data_time=[];
data_time_str=[]
data_x=[];  # xError in follower;
data_x_raw=[];  # current Pose
data_y=[];
data_y_raw=[];
data_h=[];   # headingError
data_h_rad=[];
data_h_raw=[];
data_h_raw_rad=[];
data_v_err=[];
data_v_target=[];
data_v_actual=[];
data_power=[];
max_power=0;
max_x_err=0;
max_y_err=0;
max_heading_err=0;
max_v=0;
p_name='noname';
print_summary=0;
filepath = sys.argv[1];
arg_c = len(sys.argv);

if arg_c>=3:
    print_summary = 1;

def get_time(t):
    t = t.split(' ')
    #print(t)
    t_s = ' ';
    t_s = t_s.join(t[:2])
    #print(t_s)
    t = datetime.strptime(t_s, '%m-%d  %H:%M:%S.%f')
    return t;

with open(filepath) as fp:
    line = fp.readline()
    #print(line)
    while line:
        line = fp.readline();
        #print(line)
        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and ("update: x" in line):
            t1 = line.split("update: x");
            t2 = t1[1].strip();
            t3 = t2.split(' ');
            t = t3[0];
            data_x_raw.append(float(t));

            curr_time = get_time(t1[0])
            delta = curr_time - start_time;
            data_time.append(delta.total_seconds());
            data_time_str.append(curr_time)

            end_time = curr_time;
        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and (" y " in line):
            t1 = line.split(" y ");
            t2 = t1[1].strip();
            t3 = t2.split(' ');
            t = t3[0];
            data_y_raw.append(float(t));
            #print("y: ", t);
        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and (": heading " in line):
            t1 = line.split(" heading ");
            t2 = t1[1].strip();
            t3 = t2.split(' ');
            #print(t3)
            t = t3[0];
            data_h_raw_rad.append(float(t));
            data_h_raw.append(math.degrees(float(t)));
            #print("y: ", t);

        if (("SampleMecanumDriveBase" in line) or ("BaseClass" in line)) and ("Error" in line):
            #t = line.strip();
            if ("xError" in line):
                t1 = line.split('xError')
                #print(t1)
                t2 = t1[1]
                t = float(t2)
                data_x.append(t)
                if t > max_x_err:
                    max_x_err = t;
            if ("yError" in line):
                t1 = line.split('yError')
                t2 = t1[1]
                t = float(t2)
                data_y.append(t)
                if t > max_y_err:
                    max_y_err = t;
            if ("headingError" in line):
                t1 = line.split('headingError')
                t2 = t1[1]
                data_h_rad.append(float(t2))
                t = math.degrees(float(t2));
                data_h.append(t)
                if t > max_heading_err:
                    max_heading_err = t;
        #####################################
        if ("DriveVelocityPIDTuner: error 0" in line):
            #print(t);
            t1 = line.split('error 0');
            data_v_err.append(float(t1[1]))
        if ("DriveVelocityPIDTuner: targetVelocity" in line):
            t1 = line.split('targetVelocity')
            data_v_target.append(float(t1[1]))
        if ("DriveVelocityPIDTuner: velocity 0" in line):
            t1 = line.split('velocity 0');
            t2 = t1[1];
            data_v_actual.append(float(t2))
            t = float(t2.strip());
            t = abs(t)
            if t > max_v:
                max_v = t;
        #############################################
        if ("setMotorPowers" in line) and ("leftFront" in line):
            #print(line)
            t = line.split('setMotorPowers');
            t1 = t[1].strip().split(' ');
            #print(t1)
            t2 = t1[1]
            data_power.append(float(t2))
            t3 = float(t2)
            if t3>max_power:
                max_power=t3;
                max_power_time = get_time(t[0])
                t = max_power_time-start_time
                max_power_delta = t.total_seconds()
        ###########################################
        if ("AutonomousPath: start new step: step" in line):
            print(line.rstrip())
            t = line.split('currentPos (');
            t1 = get_time(t[0]);
            last_time = t1;
            auto_time_raw.append(t1);
            t_delta = t1-start_time;
            auto_time.append(t_delta.total_seconds());

            t = t[1].split(', ');
            #print(t)
            auto_x.append(float(t[0].rstrip()));
            auto_y.append(float(t[1].rstrip()));
            t=t[2];
            t=t[:-3].strip();
            auto_h.append(float(t));
        if ("AutonomousPath: drive and builder created, initialized with pose" in line):
            print(line.rstrip())
            t = line.split('AutonomousPath');
            t1 = get_time(t[0]);
            t_delta = t1-last_time
            #print("drive reset takes: ", t_delta.total_seconds());
            create_time.append(t_delta.total_seconds());
        ###########################################
        if ("RobotCore" in line) and ("START - OPMODE" in line):
            t = line.split('OPMODE');
            t1 = t[1].strip().split(' ')
            p_name=t1[0]

            init_time = get_time(t[0])
            #print(start_time)
        if ("received command: CMD_RUN_OP_MODE" in line):
            t = line.split('CMD_RUN_OP_MODE');

            start_time = get_time(t[0])
            # print(start_time)
        if ("RobotCore" in line) and ("STOP - OPMODE" in line):
            break;
    print("xError, yError and headingError info")
    for i in range(len(data_x)):
        print(data_time_str[i], " ", data_time[i], " ", data_x[i], " ", data_x_raw[i], " ", data_y[i], " ", data_y_raw[i], " ",  data_h[i], " ", data_h_raw[i], " ",  data_h_rad[i], " ", data_h_raw_rad[i]);
    print("moving steps in autonomous");
    for i in range(len(auto_time)):
        print(auto_time_raw[i], " ", auto_time[i], " ", auto_x[i], " ", auto_y[i], " ", auto_h[i], " ", create_time[i]);

    for i in range(len(data_v_err)):
        print(data_v_err[i].strip(), " ", data_v_target[i].strip(), " ", data_v_actual[i].strip());

if print_summary != 0:
    t = len(data_x);
    if (t!=len(data_y) or t!=len(data_h) or t!=len(data_h_raw)):
        print("double check the parsing!!!", t, " ", len(data_h_raw), " ", len(data_h), " ", len(data_time));
    else:
        print("parsing looks good, len: ", t)

    plt.style.use('ggplot')
    #plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=3, mode="expand", borderaxespad=0.);
    plt.plot(data_time, data_x, label="xError");
    plt.plot(data_time, data_y, label="yError");
    plt.plot(data_time, data_h, label="headingError");
    plt.scatter(auto_time, [10 for i in range(len(auto_time))], zorder=2); # mark the drive reset;
    plt.xlabel('time(seconds)');
    plt.ylabel('inches for x, y, degrees for heading');
    plt.legend();

    plt.figure();
    plt.plot(data_time, nm.add(data_x, data_x_raw), label="target X");
    plt.plot(data_time, data_x_raw, label="actual X")
    plt.scatter(auto_time, auto_x, zorder=2)
    plt.xlabel('time (seconds)');
    plt.ylabel('distance(inches)');
    plt.legend();
    plt.figure();

    plt.plot(data_time, nm.add(data_y, data_y_raw), label="target Y");
    plt.plot(data_time, data_y_raw, label="actual Y")
    plt.scatter(auto_time, auto_y, zorder=2)
    plt.xlabel('time');
    plt.ylabel('inches');
    plt.legend();
    #plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, mode="expand", borderaxespad=0.)

    plt.figure();
    plt.plot(data_time, nm.add(data_h, data_h_raw), label="target heading");
    plt.plot(data_time, data_h_raw, label="actual heading")
    plt.xlabel('time');
    plt.ylabel('degrees');
    plt.scatter(auto_time, auto_h, zorder=2)
    plt.ylim([-30, 30])
    plt.legend();
    #plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, mode="expand", borderaxespad=0.)

    #####################################################################################################
    plt.figure();
    #plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, mode="expand", borderaxespad=0.)plt.plot(nm.add(data_x, data_x_raw), nm.add(data_y, data_y_raw), label="target path");
    plt.plot(data_x_raw, data_y_raw, label="actual path");
    plt.xlabel('X(inches)');
    plt.ylabel('Y(inches)');
    plt.scatter(auto_x, auto_y, zorder=2);
    plt.xlim([-60, 96])
    plt.ylim([-60, 60])
    plt.legend();
    plt.show()
    #####################################################################################################
    """plt.figure();
    im = plt.imread("skystone_field.png");
    implot = plt.imshow(im);

    print("===============summary==========================")

    print("data_x, data_y, data_h, data_x_raw, data_y_raw, data_heading_raw");
    print("data_v, data_v_target, data_v_actual");
    print("program : ", p_name)
    print("max power to wheel: ", max_power, " timestamp: ", max_power_time, " timeoffset: ", max_power_delta)
    print("max_x_err (inches): ", max_x_err)
    print("max_y_err (inches): ", max_y_err)
    print("max_heading_err (degrees) ", max_heading_err)
    print("max_velocity : ", max_v)
    duration = end_time - start_time;
    print("init time: ", init_time);
    print("start time: ", start_time, " end time: ", end_time, " run duration(seconds): ", duration.total_seconds());

    #print("start time(in miliseconds): ", start_time.timestamp() * 1000, " end time: ", end_time.timestamp() * 1000);

    plt.show();
    plt.close()
    """
