import matplotlib.pyplot as plt
import numpy as np
import math

class EBAND():
    def __init__(self, start, cel, utvonal, akadaly):
        self.start=start
        self.cel=cel
        
        self.utvonal=utvonal
        self.akadaly=akadaly
    
    def torlasz(self, pont):
        min_ind = 0
        min_tav = math.sqrt((pont[0] - self.akadaly[0][0])**2 + (pont[1] -self.akadaly[0][1])**2)
        for i, akt in enumerate(self.akadaly):
             tav = math.sqrt((pont[0] - akt[0])**2 + (pont[1] -akt[1])**2)
             if  tav < min_tav:
                 min_ind = i
                 min_tav = tav
        return self.akadaly[min_ind],min_tav
        
    def kozelit(self, tav):
        
        bubik=[]
        bubik.append(self.start)
        for akt, pont in enumerate(self.utvonal):
            fal, fal_tav = self.torlasz(pont)
            if akt>0 and pont!= self.cel:
                x=(self.utvonal[akt-1][0]+pont[0]+self.utvonal[akt+1][0])/3
                y=(self.utvonal[akt-1][1]+pont[1]+self.utvonal[akt+1][1])/3
                
                d1=abs((bubik[akt-1][0]-x)*(y-fal[1])-(x-fal[0])*(bubik[akt-1][1]-y))
                d1=d1/math.sqrt((bubik[akt-1][0]-x)**2+(y-bubik[akt-1][1])**2)
                
                d2=abs((self.utvonal[akt+1][0]-x)*(y-fal[1])-(x-fal[0])*(self.utvonal[akt+1][1]-y))
                d2=d2/math.sqrt((self.utvonal[akt+1][0]-x)**2+(y-self.utvonal[akt+1][1])**2)
                
                if  d1 > tav and d2 > tav:    
                     bubik.append([x,y])
                else:
                    bubik.append(pont)
           
        bubik.append(self.cel)
        return bubik  
             
def main():
    
    start = (0, 0)
    cel = (6, 6)
    akadaly= [(0, 2), (1, 3), (2, 2), (2, 4), (4, 2), (4, 3), (7, 1)]
    utvonal = [(0, 0),
     (1, 1),
     (1, 2),
     (2, 3),
     (3, 4),
     (4, 4),
     (5, 4),
     (6, 6)]
    
    x=[]
    y=[]
    for akt, pont in enumerate(utvonal):
        x.append(pont[0])
        y.append(pont[1])
    plt.plot(x,y,"k-", markersize=10)
    plt.grid()
    for i, akt in enumerate(akadaly):
        plt.plot(akt[0],akt[1],"rx",  markersize=5)
        
    tavolsag=0.1
    kozelites_maxdb = 100
    kozelites = 0.001
    
    i = 1
    arany = 1
    while (i <= kozelites_maxdb) and (arany >= kozelites):
        eband=EBAND(start, cel, utvonal, akadaly) 
        utvonal=eband.kozelit(tavolsag)
        
        x=[]
        y=[]
        if i > 1:
            elozo=szum
        szum=0
        for akt, pont in enumerate(utvonal):
            x.append(pont[0])
            y.append(pont[1])
            szum+=pont[0]**2+pont[1]**2
        plt.plot(x,y,"g-", markersize=1)
        
        if i > 1:
            arany = abs(szum - elozo)    
        i+=1
        print(arany, i)
    
    plt.plot(x,y,"r-", markersize=10)
    
    plt.show()
    
if __name__ == '__main__':
    main()