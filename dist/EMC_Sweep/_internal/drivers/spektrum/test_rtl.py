
import sys

# Make sure site-packages stays ahead of your local folder
site_packages = r"C:\Users\dagna\AppData\Roaming\Python\Python314\site-packages"
if site_packages not in sys.path:
    sys.path.insert(0, site_packages)

# Now add your local patched folder AFTER site-packages
sys.path.append(r"C:\SDR")


from pyrtlsdr_local.rtlsdr import RtlSdr


sdr = RtlSdr()
print("Center freq:", sdr.get_center_freq())
sdr.close()
