import heapq as hq
import math
import matplotlib.pyplot as plt

#g: akt költség
#h: heurisztika
#f: f=g+h
class hybrid_a_csillag:
    def __init__(self, min_x, max_x, min_y, max_y, falak, robot_hossz, kormany, lepes):

        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.falak = falak
       
        self.robot_hossz = robot_hossz
        self.kormany = kormany
        self.kormany_ktsg= [1,0,1]
        
        self.lepes = lepes
        self.lepes_ktsg = [1,0]

    def heurisztika(self, akt_poz, cel):
        return math.sqrt((akt_poz[0] - cel[0]) ** 2 + (akt_poz[1] - cel[1]) ** 2 + (math.radians(akt_poz[2]) - math.radians(cel[2])) ** 2)

    
    def utkereso(self, start, end):      

        self.start = start
        self.end = end
        
        ktsg_heap = []
        ut={}
        zs_ut={}
        
        falak = set(self.falak)
        g = 0     
        hq.heappush(ktsg_heap,(g + self.heurisztika(start, end),start))
        ut[start]=(g + self.heurisztika(start, end), start,(start,start))
    
        talalt = False
        while len(ktsg_heap)>0 and not talalt:

            akt_pont_1 =  ktsg_heap[0][1]
            akt_pont_ktsg=ktsg_heap[0][0]
            akt_pont_2=ut[akt_pont_1][1]     
            zs_ut[akt_pont_1]=ut[akt_pont_1]
            
            if self.heurisztika(akt_pont_1,end) < 1:
                
                utvonal=[end]
                akt_elem = akt_pont_1
                while akt_elem != start:
                    akt = zs_ut[akt_elem]                
                    szulo = akt[2][1]
                    utvonal.append(szulo)
                    akt_elem=akt[2][0]
                talalt=True
                
            if not talalt:
                hq.heappop(ktsg_heap)
            
                for i in range(0,3) :
                    for j in range(0,2):
                        g = akt_pont_ktsg-self.heurisztika(akt_pont_1, end)
                    
                        szomszed_x = akt_pont_2[0] + (self.lepes[j] * math.cos(math.radians(akt_pont_2[2]))) 
                        szomszed_y = akt_pont_2[1]  + (self.lepes[j] * math.sin(math.radians(akt_pont_2[2])))
                        szomszed_szog = math.degrees(math.radians(akt_pont_2[2]) + self.lepes[j] * math.tan(math.radians(self.kormany[i])/self.robot_hossz))
                    
                        szomszed_x_int = round(szomszed_x)
                        szomszed_y_int = round(szomszed_y)
                        szomszed_szog_int = round(szomszed_szog)
                    
                        szomszed = ((szomszed_x_int,szomszed_y_int,szomszed_szog_int),(szomszed_x,szomszed_y,szomszed_szog))
                    
                        if (szomszed_x_int,szomszed_y_int) not in falak and szomszed_x_int >= self.min_x and szomszed_x_int <= self.max_x and szomszed_y_int >= self.min_y and szomszed_y_int <= self.max_y:
                            h = self.heurisztika((szomszed_x_int,szomszed_y_int,szomszed_szog_int),end)
                            g = abs(self.lepes[j])+ g + self.kormany_ktsg[i] + self.lepes_ktsg[j]
                            f = h+g                                           
                            if not (szomszed[0] in ut and (f > ut[szomszed[0]][0] or szomszed[0] in zs_ut)):
                                hq.heappush(ktsg_heap,(f,szomszed[0]))
                                #plt.plot(szomszed[1][0], szomszed[1][1],"g*")
                                ut[szomszed[0]]=(f,szomszed[1],(akt_pont_1,akt_pont_2))

        return utvonal

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
    start = (0, 0, 0)
    end = (6, 4, 0)
    
    robot_hossz = 1
    
    sorok = len(bazis)
    oszlopok = len(bazis[0])
    fal=[]
    for i, sor in enumerate(bazis):
        for j, elem in enumerate(sor):
            if elem > 0:
                fal.append((i, j))
                plt.plot(i, j,"rX",  markersize=10)
    print(fal)

    plt.plot(start[0], start[1],"cs",  markersize=12)
    plt.plot(end[0], end[1],"bs",  markersize=12)
    plt.grid(True)
    plt.axis("equal")

    kormany = [-10,0,10]   
    lepes = [-1,1]
    
    csillag = hybrid_a_csillag(0, sorok-1, 0, oszlopok-1, fal, robot_hossz, kormany, lepes)
    utvonal = csillag.utkereso(start, end)

    ut_x, ut_y = [], []
    for akt in utvonal:
        ut_x.append(akt[0])
        ut_y.append(akt[1])
    print(utvonal[::-1])
    plt.plot(ut_x, ut_y, "-c")
    plt.show()


if __name__ == '__main__':
    main()