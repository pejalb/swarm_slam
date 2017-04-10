
# coding: utf-8

# In[9]:

import csv


# In[10]:

linhas=[]
with open('linhas','r') as csvfile:
    leitorLinhas = csv.reader(csvfile,delimiter=',')
    for row in leitorLinhas:
        linhas.append(row)
linhas=[[float (x) for x in l] for l in linhas]


# In[11]:

from bresenham import bresenham

ptos = [list(bresenham(int(round(linha[0])),int(round(linha[1])), int(round(linha[2])),int(round(linha[3])))) for linha in linhas]


# In[12]:

minimo=reduce(min,reduce(min,[reduce(min,l) for l in ptos]))
maximo=reduce(max,reduce(max,[reduce(max,l) for l in ptos]))
print minimo,maximo


# In[28]:

import numpy as np
ncells=1000
offset=ncells/2
mapa=np.zeros((ncells,ncells))


# In[29]:

for l in ptos:
    for idx in xrange(len(l)):
        if idx != len(l)-1:
            mapa[offset+l[idx][0]][offset+l[idx][1]]= mapa[offset+l[idx][0]][offset+l[idx][1]]+2
        else:
            mapa[offset+l[idx][0]][offset+l[idx][1]]= mapa[offset+l[idx][0]][offset+l[idx][1]]-16


# In[33]:

mapa=(1+np.exp(mapa))
mapa=1/mapa
mapa=1-mapa


# In[7]:

csvfile.close()
with open('mapa.csv', 'wb') as csvfile:
    mapawriter = csv.writer(csvfile, delimiter=' ',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for row in mapa:
        mapawriter.writerow(row)
csvfile.close()


# In[34]:

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


                
                
# In[41]:

imgplot= plt.imshow(mapa,cmap='gray')
plt.imsave('mapa.png',mapa,cmap='gray')

