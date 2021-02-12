'''
This example will list all the available/connected Kano devices and filter it
by its classes.
'''

from communitysdk import list_connected_devices,\
    RetailPixelKitSerial as PixelKit

devices = list_connected_devices()

rpk_filter = filter(lambda device: isinstance(device, PixelKit), devices)

available_rpk = list(rpk_filter)

print('Found {0} devices'.format(len(devices)))
print('Found {0} Pixel Kits'.format(len(available_rpk)))
