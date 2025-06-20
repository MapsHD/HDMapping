import os

def nmea_to_decimal(coord_str, direction):
    if not coord_str:
        return None
    degrees = int(coord_str[:2 if direction in "NS" else 3])
    minutes = float(coord_str[2 if direction in "NS" else 3:])
    decimal = degrees + minutes / 60
    return decimal if direction in "NE" else -decimal

input_folder = input("Enter the path to the input folder with nmea: ")

output_kml_path = os.path.join(input_folder, "output_all_gnmrc.kml")

placemarks = []

for filename in os.listdir(input_folder):
    if filename.lower().endswith((".nmea")):
        input_path = os.path.join(input_folder, filename)
        gnrmc_points = []

        with open(input_path, "r", encoding="utf-8", errors="ignore") as file:
            for line in file:
                if "$GNRMC" in line:
                    parts = line.strip().split(",")
                    if len(parts) < 12 or parts[2] != "A":
                        continue
                    lat = nmea_to_decimal(parts[3], parts[4])
                    lon = nmea_to_decimal(parts[5], parts[6])
                    if lat is not None and lon is not None:
                        gnrmc_points.append((lon, lat))

        if gnrmc_points:
            placemark = f"""    <Placemark>
      <name>{filename}</name>
      <LineString>
        <coordinates>\n"""
            for lon, lat in gnrmc_points:
                placemark += f"          {lon},{lat},0\n"
            placemark += """        </coordinates>
      </LineString>
    </Placemark>\n"""
            placemarks.append(placemark)

with open(output_kml_path, "w", encoding="utf-8") as kml:
    kml.write("""<?xml version="1.0" encoding="UTF-8"?>\n""")
    kml.write("""<kml xmlns="http://www.opengis.net/kml/2.2">\n""")
    kml.write("""  <Document>\n""")
    kml.write("""    <name>All routes GNRMC</name>\n""")
    for placemark in placemarks:
        kml.write(placemark)
    kml.write("""  </Document>\n</kml>\n""")

print(f"Created combined file: {output_kml_path}")
