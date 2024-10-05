def parse_gpgga(gpgga_message):
    """
    Parses a GPGGA message to extract latitude, longitude, and altitude.

    Parameters:
    gpgga_message (str): The GPGGA message string.

    Returns:
    tuple: A tuple containing latitude (float), longitude (float), and altitude (float).
    """

    # Split the GPGGA message by commas
    parts = gpgga_message.split(',')

    # Extract latitude and longitude
    lat_raw = parts[2]
    lat_dir = parts[3]
    lon_raw = parts[4]
    lon_dir = parts[5]

    # Convert latitude to decimal degrees
    lat_deg = float(lat_raw[:2])
    lat_min = float(lat_raw[2:])
    lat = lat_deg + (lat_min / 60.0)
    if lat_dir == 'S':
        lat = -lat

    # Convert longitude to decimal degrees
    lon_deg = float(lon_raw[:3])
    lon_min = float(lon_raw[3:])
    lon = lon_deg + (lon_min / 60.0)
    if lon_dir == 'W':
        lon = -lon

    # Extract altitude
    alt = float(parts[9])

    return lat, lon, alt

def parse_hdt(hdt_message):
    """
    Parses an HDT message to extract heading information.

    Parameters:
    hdt_message (str): The HDT message string.

    Returns:
    float: The heading in degrees relative to true north.
    """

    # Split the HDT message by commas
    parts = hdt_message.split(',')

    # Extract the heading (true heading) which is typically in parts[1]
    heading = float(parts[1])

    return heading
