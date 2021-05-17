import matplotlib.pyplot as plt
import numpy as np
import math

class DWA():
    def __init__(self, omega, lepes, torlasz, torlasz_iv):
        self.max_omega = math.radians(omega)
        self.min_omega = -math.radians(omega)
        
        self.max_lepes = lepes
        self.min_lepes = 0
        
        self.torlasz = torlasz
        self.torlasz_iv = torlasz_iv
        
    def dontes(self, pont, cel):
        akt=pont
        om=[]
        ktsg=[]
        i = 0
        szog_db = 100
        lepes_db = 10
        for szog in np.linspace(self.min_omega, self.max_omega, szog_db):
            omega=szog+akt[2]
            om.append(omega)
            ktsg.append(0)
            for lepes in np.linspace(self.min_lepes, self.max_lepes, lepes_db):
                x = akt[0] + lepes* math.cos(omega) 
                y = akt[1]  + lepes * math.sin(omega)
                
                ktsg[i]+=self.koltseg([x,y,omega],cel)
                #if ktsg[i] != math.inf:
                #    plt.plot(x,y,"gs",  markersize=0.1)
                #else:
                #    break
                if ktsg[i] == math.inf:
                    break
            i+=1
        min_ind = ktsg.index(min(ktsg))
        return om[min_ind]
        
    def koltseg(self, akt_pont, cel):
        ktsg = 0
        for i, akadaly in enumerate(self.torlasz):
            if math.sqrt((akt_pont[0] - akadaly[0]) ** 2 + (akt_pont[1] - akadaly[1]) ** 2) <= self.torlasz_iv:
                ktsg = math.inf
                break
        return math.sqrt((akt_pont[0] - cel[0]) ** 2 + (akt_pont[1] - cel[1]) ** 2)+ktsg
    
        
def main():
    bazis =    [[0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 1, 0, 1],
                [0, 0, 0, 0, 0],
                [0, 0, 1, 1, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0]]
    
    #x, y, irány_szög
    start = [0, 0, 0]
    end = [6, 4]
    end_iv = 0.5
    
    sorok = len(bazis)
    oszlopok = len(bazis[0])
    torlasz=[]
    torlasz_iv = 0.1
    for i, sor in enumerate(bazis):
        for j, elem in enumerate(sor):
            if elem > 0:
                torlasz.append((i, j))
                plt.plot(i, j,"rX",  markersize=10)
    print(torlasz)
    
    plt.plot(start[0], start[1],"cs",  markersize=12)
    plt.plot(end[0], end[1],"bs",  markersize=12)
    plt.grid(True)
    plt.axis("equal")
    
    omega = 70
    lepes = 3
    
    dwa=DWA(omega, lepes, torlasz, torlasz_iv)
    
    akt=start
  
    dt=0.1
    utvonal = []
    utvonal.append((start[0], start[1]))
    while math.sqrt((akt[0] - end[0]) ** 2 + (akt[1] - end[1]) ** 2) >= end_iv:
        up=dwa.dontes(akt,end)
        
        akt[0] += math.cos(up)*dt
        akt[1] += math.sin(up)*dt
        akt[2] = up
        
        utvonal.append((akt[0], akt[1]))
        plt.plot(akt[0], akt[1],"cs",  markersize=5)
    
    utvonal.append(end)   
    print(utvonal)
    plt.axis("equal")
    
    plt.show()
if __name__ == '__main__':
    main()