import re
import matplotlib.pyplot as plt

data = """
                              - time: 0
                                Position:
                                  WorldPosition:
                                    x: 89124.433683422438
                                    y: 43342.694220215068
                                    z: 3.0179884433746338
                                    r: 0.020146958027858975
                                    p: 0.029078486518490029
                                    h: 2.0912716069617701
                              - time: 0.20000000000000107
                                Position:
                                  WorldPosition:
                                    x: 89122.033020718343
                                    y: 43346.976189372675
                                    z: 3.0080347061157227
                                    r: 0.02055360135067388
                                    p: 0.029219874929813313
                                    h: 2.0866578450261843
                              - time: 0.29999999999999893
                                Position:
                                  WorldPosition:
                                    x: 89120.247370742974
                                    y: 43350.38987279321
                                    z: 2.9892349243164062
                                    r: 0.021384963016780056
                                    p: 0.029009474210661544
                                    h: 2.0708915948802362
                              - time: 0.30000000000000071
                                Position:
                                  WorldPosition:
                                    x: 89118.655928689011
                                    y: 43353.314239553561
                                    z: 2.9927246570587158
                                    r: 0.020244391520126283
                                    p: 0.029906856068867341
                                    h: 2.0680707156892146
                              - time: 0.20999999999999908
                                Position:
                                  WorldPosition:
                                    x: 89117.759031398804
                                    y: 43355.138976852235
                                    z: 3.0003259181976318
                                    r: 0.020235920795157972
                                    p: 0.029953144688762683
                                    h: 2.0561027257392817
                              - time: 0.29000000000000092
                                Position:
                                  WorldPosition:
                                    x: 89116.580445512329
                                    y: 43356.898278906097
                                    z: 3.0116536617279053
                                    r: 0.020081878363027314
                                    p: 0.030156700830754415
                                    h: 2.0886104519289215
                              - time: 0.40000000000000036
                                Position:
                                  WorldPosition:
                                    x: 89115.938833673557
                                    y: 43358.528258482183
                                    z: 3.0227131843566895
                                    r: 0.019235611039229064
                                    p: 0.030616858074027795
                                    h: 2.0547505469920049
                              - time: 0.29999999999999893
                                Position:
                                  WorldPosition:
                                    x: 89115.360601043925
                                    y: 43359.609248337365
                                    z: 3.031240701675415
                                    r: 0.018772905464135708
                                    p: 0.030726267813922559
                                    h: 2.0576145545138198
"""

def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

measurements = re.findall(r'- time: (.*?)\n\s+Position:\n\s+WorldPosition:\n\s+x: (.*?)\n\s+y: (.*?)\n', data, re.DOTALL)
results = []

for i in range(len(measurements)-1):
    time1, x1, y1 = measurements[i]
    time2, x2, y2 = measurements[i+1]
    time1, time2 = float(time1), float(time2)
    x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)

    distance = calculate_distance((x1, y1), (x2, y2))
    time_diff = time2
    speed = distance / time_diff
    results.append((distance, time_diff, speed))

for result in results:
    print(f"Distance: {result[0]}, Dt: {result[1]}, Velocity: {result[2]}")


indices = list(range(len(results)))
speeds = [result[2] for result in results]

plt.plot(indices, speeds, marker='o')
plt.xlabel('Data number')
plt.ylabel('Velocity')
plt.title('Velocity')
plt.show()
