import socket,struct,math
from subprocess import call
#control parameters
P_th=-3500;
D_th=-30;
P_v=-50;
I_v=-50;
limit=20000;
shutdown_angle=45;
v0_x=0; v0_y=0;
m1_zero=25000;
m2_zero=25000;
m3_zero=25000;
deadzone=200;
period=0.01;#sec
Tz=3;
cos_60=math.cos(math.pi/3);
sin_60=math.sin(math.pi/3)
#socket initialization
sock_measure = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
sock_measure.bind(('192.168.2.103', 6000));#Raspi inner
sock_control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
FPGA_address = ('192.168.2.102', 6000)#FPGA

sock_Ballbot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
sock_Ballbot.bind(('11.0.0.1', 6001));#Raspi outer
sock_observer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
observer_address=('11.0.0.5', 6666);#Observer

#allocate memory
buff=struct.pack('ffffffffffffffffffff',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
theta_x=0.0;  theta_y=0.0;  omg_x=0.0;    omg_y=0.0;   omg_z=0.0;
T_m1_pre=0;T_m2_pre=0;T_m3_pre=0;
q0=1;q1=0;q2=0;q3=0;
exInt=0;eyInt=0;ezInt=0;
I_m1=0;I_m2=0;I_m3=0;
KI=1000;KP=700;

v_x=0.0;      v_y=0.0;      s_x=0.0;      s_y=0.0;   
v0_m1=0;   v0_m2=0 ;  v0_m3=0;
v0_m1_I=0; v0_m2_I=0; v0_m3_I=0;
v_m1=0.0;     v_m2=0.0;     v_m3=0.0;
T_m1=0.0;     T_m2=0.0;     T_m3=0.0;
X=0;Y=0;
#test motors 
#print "Press <Enter> to test motors";
for i in range (1,101):
   dead=-i*200
   msg=struct.pack('iii',m1_zero+dead,m2_zero+dead,m3_zero+dead);
   #sock_control.sendto(msg, FPGA_address)
   #print dead;
   #raw_input()
msg=struct.pack('iii',m1_zero,m2_zero,m3_zero);
sock_control.sendto(msg, FPGA_address);#force motors to stop on beginning

buff, addr = sock_measure.recvfrom(2048)
print "The ballbot is online now!\n"
call('clear')
print "If the bot is on a plain ground, Press <Enter>"
raw_input()
#rezero
omg0_x=0; omg0_y=0; omg0_z=0;
acc0_x=0; acc0_y=0; acc0_z=0;
times=20;
for i in range(1,times+1):
    #if count%5==0:
    buff, addr = sock_measure.recvfrom(2048)
    data=struct.unpack('ffffffffffffffffffff',buff);#20 ADC
    omg0_x=omg0_x+(data[1]-data[3])/times
    omg0_y=omg0_y+(data[4]-data[3])/times
    omg0_z=omg0_z+(data[6]-data[8])/times
    acc0_x=acc0_x+(data[12]-data[11]/2)/times
    acc0_y=acc0_y+(data[13]-data[11]/2)/times
    acc0_z=acc0_z+(data[9]-data[11]/2)/times
print "Ballbot is ready now. Press <Enter> to start!";
raw_input()
#wait for queue
for i in range (1,400):
    buff, addr = sock_measure.recvfrom(2048)

count=1
print "Let's go!"
while True:
    count=count+1
    buff, addr = sock_measure.recvfrom(2048)
    data=struct.unpack('ffffffffffffffffffff',buff);#20 ADC
    #for i in range(17,20):
    #      print i,":",data[i]," ",
    #print "";
    #read data
    acc_x=(data[12]-data[11]/2-acc0_x)*0.05;
    acc_y=(data[13]-data[11]/2-acc0_y)*0.05;
    acc_z=(data[9]-data[11]/2)*0.05;
    omg_x=(data[1]-data[3]-omg0_x)*0.3;
    omg_y=(data[4]-data[3]-omg0_y)*0.3;
    omg_z=(data[6]-data[8]-omg0_z)*0.3;
    theta_x=theta_x+(omg_x+(acc_x-theta_x)/Tz)*period;
    theta_y=theta_y+(omg_y+(acc_y-theta_y)/Tz)*period;
    v_m1=-data[17]*1000000;
    v_m2=-data[18]*1000000;
 
    v_m3=-data[19]*1000000;
    if v_m1>150:
       v_m1=0
    if v_m1<-150:
       v_m1=0
    if v_m2>150:
       v_m2=0
    if v_m2<-150:
       v_m2=0

    if v_m3>150: 
       v_m3=0
    if v_m3<-150:
       v_m3=0
    gx=omg_x/180*math.pi;gy=omg_y/180*math.pi;gz=omg_z/180*math.pi;
    ax=acc_x;ay=acc_y;az=acc_z;
    mx=1;my=0;mz=0;

    norm = math.sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    #print ax,ay,az;
    norm = math.sqrt(mx*mx + my*my + mz*mz);
    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;
    #print mx,my,mz;
    # compute reference direction of flux
    hx = 2*mx*(0.5 - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2);
    hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5 - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1);
    hz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5 - q1*q1 - q2*q2);         
    bx = math.sqrt((hx*hx) + (hy*hy));
    bz = hz;     
    #print "h",hx,hy,hz,bx,bz;
    # estimated direction of gravity and flux (v and w)
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    wx = 2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
    wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
    wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5 - q1*q1 - q2*q2);  
    #print "v",vx,vy,vz;
    # error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) ;
    ey = (az*vx - ax*vz) ;
    ez = (ax*vy - ay*vx) ;
    #print "e,",ex,ey,ez;
    Kp=1;Ki=0.53;
    if not ex == 0 and not ey == 0 and not ez == 0:
       exInt = exInt + ex * Ki * period/2;
       eyInt = eyInt + ey * Ki * period/2;
       ezInt = ezInt + ez * Ki * period/2;
       # adjusted gyroscope measurements
       gx = gx + Kp*ex + exInt;
       gy = gy + Kp*ey + eyInt;
       gz = gz + Kp*ez + ezInt;
    #print q0,q1,q2,q3,

    #integrate quaternion rate and normalise
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*period/2;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*period/2;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*period/2;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*period/2;  
  
    # normalise quaternion
    norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    # print q0,q1,q2,q3,
    #theta_z = -math.atan2(2 * q1 * q2 + 2 * q0*q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/math.pi;
    theta_y = math.asin(-2 * q1 * q3 + 2 * q0*q2)* 180/math.pi; 
    theta_x = math.atan2(2 * q2 * q3 + 2 * q0*q1, -2 * q1*q1 - 2 * q2 * q2 + 1)* 180/math.pi; 

    #control algorithm

    v_x=v_m1-v_m2*cos_60-v_m3*cos_60;
    v_y=(v_m2-v_m3)*sin_60;
    v_x=v0_x-v_x;
    v_y=v0_y-v_y;
    s_x=s_x+v_x*period;
    s_y=s_y+v_y*period;
    X=P_th*theta_x+D_th*omg_x+I_v*s_x+P_v*v_x;
    Y=P_th*theta_y+D_th*omg_y+I_v*s_y+P_v*v_y;

    T_m1=X;
    T_m2=-X*cos_60+Y*sin_60;
    T_m3=-X*cos_60-Y*sin_60;
    
    #T_m1_pre=T_m1;
    #T_m2_pre=T_m2;
    #T_m3_pre=T_m3;
   # 
    #T_m1=T_m1_pre*0.95+T_m1*0.05
    #T_m2=T_m2_pre*0.95+T_m2*0.05
    #T_m3=T_m3_pre*0.95+T_m3*0.05


    #I_m1=I_m1+(m1-v_m1)*period
    #I_m2=I_m2+(m2-v_m2)*period
    #I_m3=I_m3+(m3-v_m3)*period

    #T_m1=I_m1*KI+(m1-v_m1)*KP
    #T_m2=I_m2*KI+(m2-v_m2)*KP
    #T_m3=I_m3*KI+(m3-v_m3)*KP

    #dead zone
    if T_m1>0:
       T_m1=T_m1+deadzone
    if T_m1<0:
       T_m1=T_m1-deadzone
    if T_m2>0:
       T_m2=T_m2+deadzone
    if T_m2<0:
       T_m2=T_m2-deadzone
    if T_m3>0:
       T_m3=T_m3+deadzone
    if T_m3<0:
       T_m3=T_m3-deadzone

    #over limit protection
    if T_m1>limit:
       T_m1=limit
    if T_m1<-limit:
       T_m1=-limit
    if T_m2>limit:
       T_m2=limit
    if T_m2<-limit:
       T_m2=-limit
    if T_m3>limit:
       T_m3=limit
    if T_m3<-limit:
       T_m3=-limit
    if count==10:
        count=0
        #print "acc_x,y=[","%+0.2f %0.2f" % (acc_x,acc_y),"]",
    #    print "omg_x,y=[","%+0.2f %+0.2f" % (omg_x,omg_y),"]",
#        print "theta_x,y=[","%+0.2f %+0.2f" % (theta_x,theta_y),"]"
  #      print "s_x,y=[","%+0.2f %+0.2f" % (s_x,s_y), "]",
 #       print "v_x,y=[","%+0.2f %+0.2f" % (v_x,v_y), "]",
#        print " v_m=[","%+i %+i %+i" % (v_m1,v_m2,v_m3), "]"
    show=struct.pack('fffffffff',v_m1,v_m2,v_m3,theta_x,theta_y,v_x,v_y,s_x,s_y);
    sock_observer.sendto(show,observer_address);
    
    if theta_x<shutdown_angle and theta_x>-shutdown_angle and theta_y<shutdown_angle and theta_y>-shutdown_angle:
        msg=struct.pack('iii',m1_zero+T_m1,m2_zero+T_m2,m3_zero+T_m3);
        sock_control.sendto(msg, FPGA_address)
    else:
        msg=struct.pack('iii',m1_zero,m2_zero,m3_zero);
        sock_control.sendto(msg, FPGA_address)
        break;
sock_control.close()
sock_measure.close()

