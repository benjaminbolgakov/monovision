import programs.calibratecamera
import programs.camerafeed
import programs.capturephoto
import programs.recordvideo
import programs.slamvideo
import programs.videosource

#from monov.printer import print_mainmenu
import monov.printer as printer
#printer.print_mainmenu()

def verify_config():
    # Check for existing configuration file
    config_exists = os.path.isfile('config.json')
    config = None
    if config_exists:
        with open('config.json') as conf_file:
            config = json.load(conf_file)
        printer.print_config(config)
    else:
        print("\nNo existing configuration file (config.json) found.")
        print("Create one with option '5) Configure'\n")
    return config

def run_camera_feed():
    programs.camerafeed.camera_feed()

def run_video_source():

def run_record_video():

def run_capture_photo():

def run_configure():


def menu():
    menu_options = {
        "1": ("Camera feed", run_camera_feed),
        "2": ("Video source", run_video_source),
        "3": ("Record video", run_record_video),
        "4": ("Capture photo", run_capture_photo),
        "5": ("Configure", run_configure)
    }

    while True:
        clear_screen()
        config = verify_config()
        printer.print_mainmenu(menu_options)
        choice = input("Select: ").strip().lower()

        if choice == 'q':
            break
        elif choice in menu_options:
            selected = menu_options[choice][1]
            selected()
        else:
            print("Invalid choice!")

if __name__ == "__main__":
    menu()
