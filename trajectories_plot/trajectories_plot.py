import os
import matplotlib.pyplot as plt
import laspy

folder = input("Enter the path to the folder with LAZ or CSV files (Enter = current directory): ").strip()
if not folder:
    folder = '.'

if not os.path.isdir(folder):
    print(f"Error: the given path '{folder}' does not exist or is not a directory.")
    exit(1)

files = [f for f in os.listdir(folder)
         if f.lower().endswith('.csv') or f.lower().endswith('.laz')]

print("Found files:", files)

plt.figure(figsize=(12, 10))
data_found = False

for filename in files:
    path = os.path.join(folder, filename)
    print(f"Processing file: {filename}")

    xs = []
    ys = []

    if filename.lower().endswith('.csv'):
        try:
            with open(path, 'r') as f:
                lines = f.readlines()
            for i, line in enumerate(lines):
                line = line.strip()
                if not line:
                    continue

                if ',' in line:
                    parts = line.split(',')
                else:
                    parts = line.split()

                if len(parts) < 3:
                    print(f"Line {i+1} too short: {parts}")
                    continue

                try:
                    x = float(parts[1].replace(',', '').strip())
                    y = float(parts[2].replace(',', '').strip())
                    xs.append(x)
                    ys.append(y)
                except:
                    continue 

        except Exception as e:
            print(f"Error reading CSV file {filename}: {e}")

    elif filename.lower().endswith('.laz'):
        try:
            with laspy.open(path) as las_file:
                las = las_file.read()
                xs = las.x
                ys = las.y
        except Exception as e:
            print(f"Error reading LAZ file {filename}: {e}")
            continue

    if xs and ys:
        data_found = True
        print(f"Plotting file: {filename} with {len(xs)} points")
        plt.plot(xs, ys, label=filename)

if data_found:
    plt.xlabel('(X)')
    plt.ylabel('(Y)')
    plt.title('Trajectories from CSV and LAZ files')
    plt.legend()
    plt.grid(True)
    
    output_path = os.path.join(folder, 'trajectories_plot.png')
    plt.savefig(output_path)
    print(f"Plot saved as: {output_path}")
    
    plt.show()
else:
    print("No data found to plot.")
