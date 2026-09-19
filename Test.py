import os, ctypes, sys

print("Python:", sys.version)
print("cwd:", os.getcwd())

target_dir = r"C:\Spectrum"

# 1. Try loading by bare name BEFORE adding the directory
try:
    ctypes.WinDLL("libusb-1.0.dll")
    print("libusb-1.0.dll found WITHOUT add_dll_directory (already on default search path)")
except OSError as e:
    print("libusb-1.0.dll NOT found without add_dll_directory:", e)

# 2. Now add the directory and retry
os.add_dll_directory(target_dir)
print(f"Called os.add_dll_directory({target_dir!r})")

try:
    ctypes.WinDLL("libusb-1.0.dll")
    print("libusb-1.0.dll loaded AFTER add_dll_directory -> confirms it's working")
except OSError as e:
    print("Still failing after add_dll_directory:", e)

# 3. Now try rtlsdr.dll the same way
try:
    ctypes.WinDLL("rtlsdr.dll")
    print("rtlsdr.dll loaded OK")
except OSError as e:
    print("rtlsdr.dll failed:", e)