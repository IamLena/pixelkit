import serial.tools.list_ports as list_ports
from .retailpixelkit import RetailPixelKitSerial as PixelKit

vendorIds = {
	'1027': 'rpk'
}
productIds = {
	'24597': 'rpk'
}

def list_connected_devices():
	com_list = list_ports.comports()
	connected_devices = []
	for port in com_list:
		vid = str(port.vid)
		pid = str(port.pid)
		path = port.device
		if vid in vendorIds.keys() and pid in productIds.keys():
			if productIds[pid] == 'rpk':
				rpk = PixelKit(path=path)
				rpk.connect()
				connected_devices.append(rpk)
	return connected_devices
