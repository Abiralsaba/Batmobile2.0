import glob, subprocess

def get_devices():
    devs = []
    for d in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        try:
            out = subprocess.check_output(
                ["udevadm", "info", "--query=property", "--name", d],
                text=True, timeout=2
            )
            p = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)

            vid = p.get("ID_VENDOR_ID")
            vid = f"(0x{vid})" if vid else ""

            devs.append((
                d,
                p.get("ID_SERIAL_SHORT") or p.get("ID_SERIAL", "—"),
                f'{p.get("ID_VENDOR", "—")} {vid}'.strip(),
                p.get("ID_MODEL", "—")
            ))
        except:
            devs.append((d, "—", "—", "—"))
    return devs


def main():
    devs = get_devices()
    if not devs:
        print("No serial devices found.")
        return

    print(f"{'PORT':<15} | {'SERIAL':<20} | {'VENDOR':<25} | {'PRODUCT'}")
    print("-" * 80)

    for d in devs:
        print(f"{d[0]:<15} | {d[1]:<20} | {d[2]:<25} | {d[3]}")


if __name__ == "__main__":
    main()
