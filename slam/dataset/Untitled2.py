
# coding: utf-8

# In[90]:

import re
import csv


# In[92]:

def le_arquivo_log(nome_log):
    f=open(nome_log,'r')
    f2=open(r'C:\Users\Pedro\Desktop\dataset\odom01.log','wb+')
    f3=open(r'C:\Users\Pedro\Desktop\dataset\range01.log','wb+')
    f4=open(r'C:\Users\Pedro\Desktop\dataset\bearing01.log','wb+')
    odomwriter = csv.writer(f2, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    laserwriter = csv.writer(f3, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    laserwriter2 = csv.writer(f4, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    odomwriter.writerow(r'pose_x,pose_y,pose_theta,vel_x,vel_y,vel_theta')
    #position 00 1037846044.659 +00.000 +00.000 +0.000 +0.000 +0.000 +0.000
    le_pos=re.compile(r'position\s\d{2}\s\d+\.\d+')#\s\d{2}\s\d+.\d+\s(-?+?\d+\.\d+)+\s*') 
    le_valor_pos=re.compile(r'\s[+-]+\d+\.\d{3}')
    le_laser=re.compile(r'laser\s\d{2}\s\d+\.\d+')
    le_valor_laser = re.compile(r'\d\.\d{3}\s[+-]+\d+\.\d{3}');
    for linha in f:
        #print linha
        r=[]
        ang=[]
        m=le_pos.match(linha)
        if m:
            m=le_valor_pos.findall(linha)
            odomwriter.writerow([float(x) for x in m])
        m=le_laser.match(linha)
        if m:
            m=le_valor_laser.findall(linha)
            v=[[float(x) for x in e.split(' ')] for e in m]
            for linha in v:
                r.append(linha[0])
                ang.append(linha[1])
                #print r
            laserwriter.writerow(r)
            laserwriter2.writerow(ang)
    f.close()
    f2.close()
    f3.close()
    f4.close()


# In[93]:

le_arquivo_log(r'C:\Users\Pedro\Desktop\dataset\run01.log')

