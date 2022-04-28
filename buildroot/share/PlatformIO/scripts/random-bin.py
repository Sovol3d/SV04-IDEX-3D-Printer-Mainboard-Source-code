#
# random-bin.py
# Set a unique firmware name based on current date and time
#
Import("env")

from datetime import datetime

env['PROGNAME'] = datetime.now().strftime("firmware")
