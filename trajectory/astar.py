import matplotlib.pyplot as plt

class pont():
    def __init__(self, szulo, poz):
        self.szulo = szulo
        self.poz = poz

        #g: költség
        #h: heurisztika
        #f: f=g+h

class csillag():
    def __init__(self, bazis, start, end):
        self.bazis=bazis
        self.start=start
        self.end=end
        
    def utkereso(self):
    
        start_pont = pont(None, self.start)
        start_pont.g = start_pont.h = start_pont.f = 0

        end_pont = pont(None, self.end)
        end_pont.g = end_pont.h = end_pont.f = 0

        sorok = len(self.bazis)
        oszlopok = len(self.bazis[0]) 
        
        utvonal=[]
        utvonal.append(start_pont)

        z_ut=[]


        while len(utvonal) > 0:
            i = 0
            akt_pont = utvonal[i]

            for index, elem in enumerate(utvonal):
                if elem.f < akt_pont.f:
                    akt_pont = elem
                    i = index
                    
            utvonal.pop(i)
            z_ut.append(akt_pont)

            if akt_pont.poz == end_pont.poz:
                ok_ut = []
                vegso = akt_pont
        
                while vegso is not None:
                    ok_ut.append(vegso.poz)
                    vegso = vegso.szulo
                return ok_ut

            gyerek = []
            

            for j in range(-1,2):
                for k in range(-1,2):
                    if not((j == 0) and (k == 0)):
                        poz = (akt_pont.poz[0]+j,akt_pont.poz[1]+k)
                        if (poz[0]  >= 0 and poz[0]  <= sorok - 1) and (poz[1]  >= 0 and poz[1]  <= oszlopok- 1) and (self.bazis[poz[0]][poz[1]] == 0):
                            uj_pont = pont(akt_pont, poz)
                            uj_pont.g = akt_pont.g + 1
                            uj_pont.h =  ((uj_pont.poz[0] - end_pont.poz[0]) ** 2) + ((uj_pont.poz[1] - end_pont.poz[1]) ** 2) ** .5
                            uj_pont.f = uj_pont.g + uj_pont.h
                            
                            gyerek.append(uj_pont)
           
            for gy_akt in gyerek:
                for akt in z_ut:
                    if gy_akt.poz == akt.poz:
                        continue

                for akt in utvonal:
                    if (gy_akt.poz == akt.poz) and (gy_akt.f > akt.f):
                        continue

                utvonal.append(gy_akt)
                        

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

    start = (0, 0)
    end = (6, 4)
    
    for i, sor in enumerate(bazis):
        for j, elem in enumerate(sor):
            if elem > 0:
                plt.plot(i, j,"rX",  markersize=10)

    plt.plot(start[0], start[1],"cs",  markersize=12)
    plt.plot(end[0], end[1],"bs",  markersize=12)
    plt.grid(True)
    plt.axis("equal")
    
    csill = csillag(bazis, start, end)
    utvonal=csill.utkereso()
    
    ut_x, ut_y = [], []
    for akt in utvonal:
        ut_x.append(akt[0])
        ut_y.append(akt[1])
    print(utvonal[::-1])
    plt.plot(ut_x, ut_y, "-c")
    plt.show()


if __name__ == '__main__':
    main()

