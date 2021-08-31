import numpy as np

posicao_grip= np.array([0.03687640279531479, -2.0855307579040527, 0.11951282620429993])
posicao_cubo= np.array([0, -2.0, 0.04999992251396179])
d=np.sqrt((posicao_grip[0]-posicao_cubo[0])**2+(posicao_grip[1]-posicao_cubo[1])**2+(posicao_grip[2]-posicao_cubo[2])**2)

print(d)