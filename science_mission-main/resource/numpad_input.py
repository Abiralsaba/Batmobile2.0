#!/usr/bin/env python3
"""
HID Keypad Listener Node (Robot State Version)
Reads and prints raw key events from a dedicated input device.
"""

import sys
import signal
import select
from evdev import InputDevice, categorize, ecodes, KeyEvent, list_devices

BUS_USB = 0x03  # linux/input.h — USB bus type

class KeypadReader:
    _REJECT_KEYWORDS = (
        "at translated", "thinkpad", "internal", "laptop",
        "macbook", "chromebook", "built-in",
        "power button", "video bus", "pc speaker", "sleep button",
        "lid switch", "virtual", "sysrq",
        "system control", "consumer control", 
    )

    def __init__(self, device_path=None, grab=True):
        self._grabbed = False
        self._running = True
        self._grab_requested = grab
        self._ctrl_held = False
        self.device = None

        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self.device_path = device_path or self._auto_detect_device()
        if not self.device_path:
            print("[ERROR] Could not automatically find a keypad. Please check USB connection.")
            sys.exit(1)
            
        # Robot State Array (10 elements)
        # 0: Power (Toggle, key /)
        # 1: DS    (Toggle, key *)
        # 2: BS    (Toggle, key .)
        # 3: HMS   (Left/Right, key 0, False=Left)
        # 4: Fire  (Toggle, key ENTER)
        # 5: Light (Toggle, key 5)
        # 6: Up    (Continuous, key 8)
        # 7: Down  (Continuous, key 2)
        # 8: Left  (Continuous, key 4)
        # 9: Right (Continuous, key 6)
        self.state = [False] * 10
            
    def _signal_handler(self, signum, frame):
        sig_name = signal.Signals(signum).name
        print(f"\n[INFO] Received {sig_name}. Stopping...")
        self._running = False

    def _auto_detect_device(self):
        print("[INFO] Auto-scanning for USB keypads...\n")
        candidates = []

        for path in list_devices():
            try:
                dev = InputDevice(path)
            except Exception:
                continue

            name = dev.name
            low = name.lower()
            phys = (dev.phys or "").lower()
            bustype = dev.info.bustype

            if bustype != BUS_USB or "usb" not in phys or ecodes.EV_KEY not in dev.capabilities():
                dev.close()
                continue

            if "mouse" in low or any(kw in low for kw in self._REJECT_KEYWORDS):
                dev.close()
                continue

            score = 10
            if "keypad" in low or "numpad" in low:
                score = 100
            elif "hid" in low:
                score = 50

            candidates.append((score, path, name))
            dev.close()

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[0], reverse=True)
        best = candidates[0]
        return best[1]

    def _print_state(self, message=""):
        # Format the 10 boolean states as a continuous binary string e.g., "1000101000"
        state_str = "".join(["1" if x else "0" for x in self.state])
        if message:
            print(f"{message:<25} | Data: {state_str}")
        else:
            print(f"Data: {state_str}")

    def run(self):
        try:
            self.device = InputDevice(self.device_path)
            print(f"\n[INFO] Connected to: {self.device.name} ({self.device_path})")
            
            if self._grab_requested:
                self.device.grab()
                self._grabbed = True
                print("[INFO] Device GRABBED (exclusive access).")

            print("[INFO] Listening for key presses... (Press Ctrl+C to exit)\n")

            fd = self.device.fd

            while self._running:
                r, w, x = select.select([fd], [], [], 0.1)  # Wake up every 100ms
                if not r:
                    continue

                for event in self.device.read():
                    if event.type == ecodes.EV_KEY:
                        key_event = categorize(event)
                        
                        keycode = key_event.keycode
                        if isinstance(keycode, list):
                            keycode = keycode[0]
                            
                        # ── Track Ctrl key state (for Ctrl+C / Ctrl+Z detection) ──
                        if keycode in ("KEY_LEFTCTRL", "KEY_RIGHTCTRL"):
                            if key_event.keystate in (KeyEvent.key_down, KeyEvent.key_hold):
                                self._ctrl_held = True
                            elif key_event.keystate == KeyEvent.key_up:
                                self._ctrl_held = False
                            continue

                        # ── Detect Ctrl+C or Ctrl+Z in the event stream itself ────
                        if key_event.keystate == KeyEvent.key_down and self._ctrl_held:
                            if keycode == "KEY_C":
                                print("\n[INFO] Ctrl+C detected. Exiting...")
                                self._running = False
                                return
                            if keycode == "KEY_Z":
                                print("\n[INFO] Ctrl+Z detected. Exiting...")
                                self._running = False
                                return

                        is_press = (key_event.keystate == KeyEvent.key_down)
                        is_release = (key_event.keystate == KeyEvent.key_up)
                        state_changed = False
                        msg = ""

                        # --- TOGGLE KEYS (Trigger ON/OFF only when initially pressed) ---
                        if is_press:
                            if keycode in ["KEY_KPSLASH"]:  # / -> Power
                                self.state[0] = not self.state[0]
                                msg = f"Power {'ON' if self.state[0] else 'OFF'}"
                                state_changed = True
                                
                            elif keycode in ["KEY_KPASTERISK"]:  # * -> DS
                                self.state[1] = not self.state[1]
                                msg = f"DS {'ON' if self.state[1] else 'OFF'}"
                                state_changed = True
                                
                            elif keycode in ["KEY_KPDOT", "KEY_DELETE"]:  # . -> BS
                                self.state[2] = not self.state[2]
                                msg = f"BS {'ON' if self.state[2] else 'OFF'}"
                                state_changed = True
                                
                            elif keycode in ["KEY_KP5", "KEY_CLEAR"]:  # 5 -> Light
                                self.state[5] = not self.state[5]
                                msg = f"Light {'ON' if self.state[5] else 'OFF'}"
                                state_changed = True
                                
                            elif keycode in ["KEY_KPENTER", "KEY_ENTER"]:  # ENTER -> Fire
                                self.state[4] = not self.state[4]
                                msg = f"Fire {'ON' if self.state[4] else 'OFF'}"
                                state_changed = True
                                
                            elif keycode in ["KEY_KP0", "KEY_INSERT"]:  # 0 -> HMS (Left/Right)
                                if not self.state[4]: # Only toggle HMS if Fire is OFF
                                    self.state[3] = not self.state[3]
                                    msg = f"HMS {'Right' if self.state[3] else 'Left'}"
                                else:
                                    msg = "HMS Blocked (Fire is ON!)"
                                state_changed = True

                            # --- CONTINUOUS KEYS (Trigger ON when pressed) ---
                            elif keycode in ["KEY_KP2", "KEY_DOWN"]:
                                self.state[7] = True
                                msg = "Down move"
                                state_changed = True
                                
                            elif keycode in ["KEY_KP8", "KEY_UP"]:
                                self.state[6] = True
                                msg = "Up move"
                                state_changed = True
                                
                            elif keycode in ["KEY_KP4", "KEY_LEFT"]:
                                self.state[8] = True
                                msg = "Video left move"
                                state_changed = True
                                
                            elif keycode in ["KEY_KP6", "KEY_RIGHT"]:
                                self.state[9] = True
                                msg = "Video right move"
                                state_changed = True

                        # --- CONTINUOUS KEYS (Trigger OFF when released) ---
                        elif is_release:
                            if keycode in ["KEY_KP2", "KEY_DOWN"]:
                                self.state[7] = False
                                msg = "Down Stop"
                                state_changed = True
                                
                            elif keycode in ["KEY_KP8", "KEY_UP"]:
                                self.state[6] = False
                                msg = "Up Stop"
                                state_changed = True
                                
                            elif keycode in ["KEY_KP4", "KEY_LEFT"]:
                                self.state[8] = False
                                msg = "Video left stop"
                                state_changed = True
                                
                            elif keycode in ["KEY_KP6", "KEY_RIGHT"]:
                                self.state[9] = False
                                msg = "Video right stop"
                                state_changed = True

                        # Generate String and Print data if any valid key changed state
                        if state_changed:
                            self._print_state(msg)

        except PermissionError:
            print(f"\n[ERROR] Permission denied!")
            print(f"        Linux blocks standard users from reading hardware inputs.")
            print(f"        You must be in the 'input' group to run without sudo.")
        except FileNotFoundError:
            print(f"\n[ERROR] Device {self.device_path} not found. Check if the keypad is plugged in.")
        except KeyboardInterrupt:
            pass  # Handled by signal handler now
        except OSError as e:
            if self._running:
                print(f"\n[ERROR] Device error: {e}")
        finally:
            self._cleanup()

    def _cleanup(self):
        if self.device is not None:
            try:
                if self._grabbed:
                    self.device.ungrab()
                    self._grabbed = False
            except OSError:
                pass
            try:
                self.device.close()
            except Exception:
                pass


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="HID Keypad Listener (Robot State)")
    parser.add_argument("-d", "--device", default=None, help="Explicit path.")
    parser.add_argument("--no-grab", action="store_true", default=False, help="Do not grab exclusive access.")
    args = parser.parse_args()

    # grab is True by default, unless --no-grab is specified
    reader = KeypadReader(device_path=args.device, grab=(not args.no_grab))
    reader.run()
