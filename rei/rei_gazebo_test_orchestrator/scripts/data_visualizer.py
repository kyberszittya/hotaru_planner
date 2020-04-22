import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def main():
    p = pd.read_csv("./data/datacollection_2020_04_22_20_30_35.csv", sep=';')
    sns.distplot(p['abs_lateral_distance'], kde=False, rug=True)
    plt.figure()
    plt.plot(p['wheel_angle'])
    plt.figure()
    plt.plot(p['linear_velocity'])
    plt.show()

if __name__=="__main__":
    main()