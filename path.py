import math
import heapq
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

#dict a keresesi ido O(1) gyorsan visszaadja a magassag-akadaly kombot
#gyorsan tudom kerensi a sozmszedokat
coord = {}
#make happen


with open('surface_100x100.txt') as f:
    for line in f:
        data = line.split()
        x = int(data[0])
        y = int(data[1])
        z = float(data[2])
        b = int(data[3])
        coord[(x, y)] = (z, b)

#kezdo es veg
with open('surface_100x100.end_points.txt') as f:
    startX, startY = map(int, f.readline().split())
    endX, endY = map(int, f.readline().split())
print(endX,' ',endY)

end_magassag = coord[(endX, endY)][0]

print(f"A vegpont magassaga: {end_magassag}")
#euklideszi tav
def eu_tavolsag(x1, y1, z1, x2, y2, z2):
    tavolsag = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
    return tavolsag

def manhattan_tavolsag(x1, y1, x2, y2):
    return max(abs(x2 - x1), abs(y2 - y1))

#tavolsag jelenelgi csmp-tol a vegosig
#csak becsuli nem a jelenlegi
#becsult hatralevi
def h(n):
    return abs(n[0] - endX) + abs(n[1] - endY)

#kezdoponttol a jelenlegi pontig
#eddig megtett kolt
def g(n):
    return abs(n[0] - startX) + abs(n[1] - startY)

# A* algoritmus
def A(use_euclidean=False):
    #ha true figyelembe veszi a magassagot is
    start = (startX, startY)
    goal = (endX, endY)


    varakozas = []  #prior sor
    feldolgozott = []  #fel csmp
    innen_jott = {}  # #mleyik csmp volt a szuloije

    valodi_kolt = {start: 0}

    heapq.heappush(varakozas, (0, start))  #adott csmp-thoz a kotlseg

    while varakozas:
        #lgegk oskz csmp
        f, current = heapq.heappop(varakozas)

         #ameddig van fel nem dolgozogtt csmp
        if current == goal:
             #megnezsi ha elertuk a celt
            #kezdoponttol vegpontig
            path = []
            while current in innen_jott:
                 #hozzaadja a csmp utvonalhoz
                path.append(current)
                #kovi csmp
                current = innen_jott[current]
             #kezdopont nem szerepel az innen jottbe
            path.append(start)
            #hat hogyjo sorrendbe legyen
            path.reverse()
            return path, valodi_kolt

        feldolgozott.append(current)

        #szomszedos csmp feldolgozasa
        #-1 -1:bal felso,-1 1: jobb felso,-1 0 fel,1 0:le ......
        for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
             #kovi szomszeodsz pont
            nx, ny = current[0] + dx, current[1] + dy

            #megnezme ha letezik e
            if (nx, ny) in coord:
                z, b = coord[(nx, ny)] #lekerem a magassagot es ha vanm e akadaly

                #olyan kell amin nincs akadaly
                if b == 0:
                    if (nx, ny) in feldolgozott:
                        continue
                     #mer 1 ravolsagra van
                     # eddig mgetett ut
                    uj_koltseg = valodi_kolt[current] + (eu_tavolsag(nx, ny, z, current[0], current[1], coord[(current[0], current[1])][0]) if use_euclidean else 1)


                    #ut htatralevee reszeee
                    if use_euclidean:
                        heuristic = eu_tavolsag(nx, ny, z, endX, endY, end_magassag)
                    else:
                        heuristic = manhattan_tavolsag(nx, ny, endX, endY)

                    #hozzaadjaa prior sorhoz az uju csmpot ha jobb a koltsge
                    #ha nicns benne a megtett legjobb kopltsegek kozott, ha tartozik mar koltseg a csompontohoz ami kisebb
                    if (nx, ny) not in valodi_kolt or uj_koltseg < valodi_kolt[(nx, ny)]:
                        valodi_kolt[(nx, ny)] = uj_koltseg
                        #ez mgaga az alg ertefs
                        ossz_koltseg = uj_koltseg + heuristic

                        #uj csmop hozzaad prior sorhoz
                        heapq.heappush(varakozas, (ossz_koltseg, (nx, ny)))
                        innen_jott[(nx, ny)] = current  #szomszeods csmp szuloje a current

    return None, valodi_kolt



def w_falj(utvonal1, valodi_kolt1, utvonal2=None, valodi_kolt2=None):
    with open("output.txt", "w") as f:
        f.write("Elso: \n")
        for csomopont in utvonal1:
            f.write(f"{csomopont} ")
        f.write(f"\nKoltseg: {valodi_kolt1[(endX, endY)]}\n\n")

        if utvonal2:
            f.write("Masodik: \n")
            for csomopont in utvonal2:
                f.write(f"{csomopont} ")
            f.write(f"\nKoltseg: {valodi_kolt2[(endX, endY)]}\n")

#elso min hossz ut
elso_utvonal, valodi_kolt = A(use_euclidean=True)
masodik_utvonal, valodi_kolt2 = A(use_euclidean=False)

w_falj(elso_utvonal, valodi_kolt, masodik_utvonal, valodi_kolt2)


#mat
heatmap=np.zeros((100, 100))

for (x, y), (z, b) in coord.items():
    #megf magassat
    heatmap[y, x] = coord[(x, y)][0]


plt.imshow(heatmap, cmap="inferno", interpolation='nearest')

#akadaly es utvonal
for (x, y), (z, b) in coord.items():
    if b == 1:
        #ak piros szinm kor
        plt.plot(x, y, "ro")
    elif (x, y) in elso_utvonal:
        plt.plot(x, y, "go")
    elif (x, y) in masodik_utvonal:
        plt.plot(x, y, "bo")
plt.title("2D Hoterkep")
plt.colorbar()
plt.show()

###################3

#ures abra
fig = plt.figure()
#tengelyek
ax = fig.add_subplot(111, projection='3d')

x = []
y = []
z = []
c = []
szinek = []

#ak
akx,ay,az = ([], [], [])

for (coordX, coordY), (magassag, b) in coord.items():
    x.append(coordX)
    y.append(coordY)
    z.append(magassag)
    if b == 1:
        akx.append(coordX)
        ay.append(coordY)
        az.append(magassag)
    #ami nem akaddaly
    else:
      c.append('yellow')

#pont siznai magasas
ax.scatter(np.array(x), np.array(y), np.array(z), c = z, cmap="viridis")
ax.plot(np.array(akx), np.array(ay), np.array(az), "ro")
#utvonal
if elso_utvonal:
    ut_x = [p[0] for p in elso_utvonal]
    ut_y = [p[1] for p in elso_utvonal]
    ut_z = [coord[(p[0], p[1])][0] for p in elso_utvonal]

    ax.plot(ut_x, ut_y, ut_z, "-bo" )
    # '''color='blue', linewidth=2'''

if masodik_utvonal:
    ut_x2 = [p[0] for p in masodik_utvonal]
    ut_y2 = [p[1] for p in masodik_utvonal]
    ut_z2 = [coord[(p[0], p[1])][0] for p in masodik_utvonal]

    ax.plot(ut_x2, ut_y2, ut_z2, "-co")

ax.set_xlabel('X teng')
ax.set_ylabel('Y teng')
ax.set_zlabel('Magasszg')
plt.title("3D")
plt.legend()
plt.show()


