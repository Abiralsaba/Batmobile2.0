import os

serial_map = {
    'ARM_Serial' : '16832890',
    'BiFrost_Serial' : '16832280',
    'Test_Serial' : '16832500',
    'Dummy' : '16332200',
}


def get_tty_from_serial(serial_number):
    def find_tty_in_device_tree(device_path):
        try:
            for subdir in os.listdir(device_path):
                if ":" in subdir:
                    tty_class_path = os.path.join(device_path, subdir, "tty")
                    if os.path.exists(tty_class_path):
                        if tty_names := os.listdir(tty_class_path):
                            return f"/dev/{tty_names[0]}"
        except (OSError, IOError):
            pass
        return None

    base_path = "/sys/bus/usb/devices/"
    for device_dir in os.listdir(base_path):
        device_path = os.path.join(base_path, device_dir)
        serial_file = os.path.join(device_path, "serial")
        if os.path.exists(serial_file):
            try:
                with open(serial_file, 'r') as f:
                    if f.read().strip() == serial_number:
                        return find_tty_in_device_tree(device_path)
            except (OSError, IOError):
                continue
    return None


target_serial = serial_map['ARM_Serial']
device_name = get_tty_from_serial(target_serial)


if device_name:
    print(f"Device with serial {target_serial} is at {device_name}")
else:
    print(f"No device found with serial {target_serial}")
