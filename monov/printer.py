# ANSI Style Codes
RESET = "\033[0m"
BOLD = "\033[1m"
UL = "\033[4m"

# ANSI Foreground Colors
GREEN = "\033[32m"
RED = "\033[31m"
CYAN = "\033[36m"
YELLOW = "\033[33m"

separator = "================================================="

def print_mainmenu(options):
    print(f"\n{BOLD}{UL}{GREEN}___ Monovision ___\n{RESET}")
    for key, (label, _) in options.items():
        print(f"{key}) {label}")
    print("q) Exit\n")

def print_config(config):
    print("\n=Current configuration=")
    print(f"Calibration file: {config['calibration_file']}")
    print(f"Camera resolution: {config['camera_resolution'][0]}x{config['camera_resolution'][1]}")
    print(f"Marker size: {config['marker_size']}mm\n")

def printer(data_package, f_name):
    write_package = []
    print(separator)
    write_package.append(separator)
    for data in data_package:
        print(data)
        write_printer(data, f_name)
    write_package.append(separator)
    print(separator)

def write_printer(data, f_name):
    with open(log_dir+f_name, "a") as file:
        file.write(str(data) + "\n")
