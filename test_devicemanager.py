from communitysdk import list_connected_devices
from communitysdk import RetailPixelKitSerial as PixelKit
from types import SimpleNamespace
import pytest
from unittest.mock import patch

def test_list_connected_devices():
	'''
	Should return only connected devices with Kano's vendor and product ids
	'''
	vDevices = [
		SimpleNamespace(vid=None, pid=None, device="UNKNOWN"),
		SimpleNamespace(vid=1027, pid=24597, device="RPK")
	]
	with patch('communitysdk.RetailPixelKitSerial.serial_connect'),\
		patch('serial.tools.list_ports.comports', return_value=vDevices):
		devices = list_connected_devices()
		assert len(devices) == 1
		for d in devices:
			d.close()

def test_list_connected_devices_instances():
	'''
	Should return connected instances of devices
	'''
	vDevices = [
		SimpleNamespace(vid=1027, pid=24597, device="RPK")
	]
	with patch('communitysdk.RetailPixelKitSerial.serial_connect'),\
		patch('serial.tools.list_ports.comports', return_value=vDevices):
		devices = list_connected_devices()
		assert isinstance(devices[0], PixelKit)
		assert devices[0].is_connected == True
		for d in devices:
			d.close()

test_list_connected_devices()
test_list_connected_devices_instances()