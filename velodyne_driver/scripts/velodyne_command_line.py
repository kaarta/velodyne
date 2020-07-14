#!/usr/bin/python3

import pycurl

try:
    from io import BytesIO
except ImportError:
    from StringIO import StringIO as BytesIO
try:
    from urllib.request import urlopen
    from urllib.parse import urlencode
except ImportError:
    from urllib import urlencode
import argparse

# import urllib2
import json
import time

from socket import *
import sys

def pretty_print(data):
    print(json.dumps(data, sort_keys=True, indent=4, separators=(',', ': ')))

class VelodyneConfigurator:
    ip_addr = "192.168.1.201"
    data_port = 2368
    Base_URL = 'http://192.168.1.201/cgi/'

    sensor = pycurl.Curl()
    buffer = BytesIO()
    is_streaming_data = False
    snapshot_beginning = []
    verbose = False
    quiet = False

    def __init__(self, args):
        ip_addr = args.ip_addr
        self.verbose = args.verbose
        self.quiet = args.quiet
        if (self.quiet):
            self.verbose = False

        self.ip_addr = self.detect_ip(ip_addr, self.verbose, self.quiet)
        self.Base_URL = 'http://'+str(self.ip_addr)+'/cgi/'
        if (self.verbose):
            print("Base url: " + str(self.Base_URL))

    def detect_ip(self, ip_addr='auto', verbose=False, quiet=False):
        if (ip_addr == 'auto'):
            ip_addr = ""

        try:
            s=socket(AF_INET, SOCK_DGRAM)
            s.settimeout(1.0)
            s.bind((ip_addr,self.data_port))
            if (verbose):
                print("Waiting for data on port: " + str(self.data_port))
            data,address=s.recvfrom(1024)
            if (len(data) > 0):
                address = address[0]
                if (verbose):
                    print("Velodyne packet data is coming from: " + str(address))
                self.is_streaming_data = True
                if (len(ip_addr) == 0):
                    ip_addr = address
        except Exception as ex:
            if (not quiet):
                print("Could not determine velodyne IP address\nException: " + str(ex))
            # if (len(ip_addr) == 0):
            #     exit(1)
            self.is_streaming_data = False

        return ip_addr



    def sensor_do(self, url, key_values, buf):
        self.sensor.setopt(self.sensor.URL, self.Base_URL + url)
        self.sensor.setopt(self.sensor.POSTFIELDS, key_values)
        self.sensor.setopt(self.sensor.WRITEDATA, buf)
        self.sensor.perform()
        rcode = self.sensor.getinfo(self.sensor.RESPONSE_CODE)
        success = rcode in range(200, 207)
        if (self.verbose):
            print('%s %s: %d (%s)' % (url, key_values, rcode, 'OK' if success else 'ERROR'))
        return success, rcode

    def get_json(self, url, timeout=10):
        response = urlopen(self.Base_URL+url,timeout=timeout)
        if (response):
            return (True, json.loads(response.read()))
        else:
            return (False, [])

    def setup_settings(self, laser_state="on", rpm="600", returns="Strongest"):
        settings = {'laser':'on', 
                    'rpm':'600',
                    'returns': 'Strongest'}

        rc, code = self.sensor_do('setting', urlencode(settings), self.buffer)
        if (rc):
            if self.verbose:
                print("Set settings to: " + str(settings))
        else:
            if (not self.quiet):
                print("Failed to set settings to: " + str(settings) +" ... error code = " + str(code))
        return rc

    def setup_fov(self, start=0, end=359, pwr_save="off"):
        settings  = { 'start': str(start),
                      'end': str(end),
                      'pwrsaven': pwr_save }

        rc, code = self.sensor_do('setting/fov', urlencode(settings), self.buffer)
        if (rc):
            if self.verbose:
                print("Set FOV to: " + str(settings))
        else:
            if (not self.quiet):
                print("Failed to set FOV to: " + str(settings) +" ... error code = " + str(code))
            return False
        return rc

    def setup_host(self, addr="10.20.27.1", dport="self.data_port", tport="8308", udp_checksum="off"):
        settings = {"addr": addr,
                    "dport": dport,
                    "tport": tport,
                    "udpcs": udp_checksum}

        rc, code = self.sensor_do('setting/host', urlencode(settings), self.buffer)
        if (rc):
            if self.verbose:
                print("Set host to: " + str(settings))
        else:
            if (not self.quiet):
                print("Failed to set host to: " + str(settings) +" ... error code = " + str(code))
        return rc

    def setup_net(self, addr="10.20.27.4", mask="255.255.255.0", gateway="10.20.27.1", dhcp="off", mac_addr="00-00-00-00-00-00"):
        settings = {"addr": addr,
                    "mask": mask,
                    "gateway": gateway,
                    "dhcp": dhcp,
                    "mac_addr": mac_addr}

        rc, code = self.sensor_do('setting/net', urlencode(settings), self.buffer)
        if (rc):
            if self.verbose:
                print("Set net to: " + str(settings))
        else:
            if (not self.quiet):
                print("Failed to set net to: " + str(settings) +" ... error code = " + str(code))
        return rc

    def setup_gps_qualifier(self, gps_valid="off"):
        settings = {"gpsrxv": gps_valid}

        rc, code = self.sensor_do('setting/gpsctl', urlencode(settings), self.buffer)
        if (rc):
            if self.verbose:
                print("Set GPS Qualifier to: " + str(settings))
        else:
            if (not self.quiet):
                print("Failed to set GPS Qualifier to: " + str(settings) +" ... error code = " + str(code))
        return rc

    def setup_pps_qualifier(self, require_gps_valid = "off", ppslck = "off", delay = "5"):
        settings = {"gpsrxv": require_gps_valid,
                    "ppslck": ppslck,
                    "delay": delay}

        rc, code = self.sensor_do('setting/ppsctl', urlencode(settings), self.buffer)
        if (rc):
            if self.verbose:
                print("Set PPS settings to: " + str(settings))
        else:
            if (not self.quiet):
                print("Failed to set PPS settings to: " + str(settings) +" ... error code = " + str(code))
        return rc

    def setup_phase_lock(self, enabled="off", offset = "0", offsetInput = "0"):
        settings = {"enabled": enabled,
                    "offset": offset,
                    "offsetInput": offsetInput}

        rc, code = self.sensor_do('setting/phaselock', urlencode(settings), self.buffer)
        if (rc):
            if self.verbose:
                print("Set PPS settings to: " + str(settings))
        else:
            if (not self.quiet):
                print("Failed to set PPS settings to: " + str(settings) +" ... error code = " + str(code))
        return rc

    # Setting options are "on/off" for enabled, and "Reduced" or "Full" per group
    def setup_laser_power_save(self, enabled="off", top_group = "Full", highres_group="Full", bottom_group="Full"):
        settings = {"en": enabled,
                    "sky": top_group,
                    "hires": highres_group,
                    "gnd": bottom_group}

        rc, code = self.sensor_do('setting/powersave', urlencode(settings), self.buffer)
        if (rc):
            if self.verbose:
                print("Set power settings to: " + str(settings))
        else:
            if (not self.quiet):
                print("Failed to set power settings to: " + str(settings) +" ... error code = " + str(code))
        return rc

    def save(self):
        rc, code = self.sensor_do('save', urlencode({'data': 'submit'}), self.buffer)
        if (rc):
            if self.verbose:
                print("Saved settings to NVRAM")
        else:
            if (not self.quiet):
                print("Failed to save settings to NVRAM ... error code = " + str(code))
        return rc


    def restart(self):
        rc, code = self.sensor_do('reset', urlencode({'data': 'reset_system'}), self.buffer)
        if (rc):
            if self.verbose:
                print("Restarting sensor...")
        else:
            if (not self.quiet):
                print("Failed to restart sensor ... error code = " + str(code))
        return rc

    def setup_stencil_pro_velodyne(self):
        if not self.setup_settings(): return 1
        time.sleep(1)
        if not self.setup_fov(): return 1
        time.sleep(1)
        if not self.setup_pps_qualifier("off", "on", "5"): return 1
        time.sleep(1)
        if not self.setup_gps_qualifier(): return 1
        time.sleep(1)
        if not self.setup_phase_lock(): return 1
        time.sleep(1)
        if not self.setup_laser_power_save(): return 1
        time.sleep(1)
        if not self.setup_host(): return 1
        time.sleep(1)
        if not self.setup_net(): return 1
        time.sleep(2)

        if not self.save(): return 1
        time.sleep(1)
        if not self.restart(): return 1
        new_ip = ""
        wait_count = 30
        while(len(new_ip) == 0 and wait_count > 0):
            if (self.verbose):
                print("Waiting for system to come back up")

            wait_count-=1
            time.sleep(1)
            new_ip = self.detect_ip(verbose=False, quiet = True)

        if (len(new_ip) > 0):
            self.ip_addr = new_ip

            if (self.verbose):
                print("Got velodyne data from IP address: " + str(self.ip_addr) + " and port: " + str(self.data_port))

            self.Base_URL = 'http://'+str(self.ip_addr)+'/cgi/'
            if (self.verbose):
                print("New base url: " + str(self.Base_URL))

            (response, status) = self.get_json('snapshot.hdl')
            if response:
                print("Laser configured for\nKaarta Stencil Pro usage")
                print("Model: " + str(status['info']['model']))
                print("Serial: " + str(status['info']['serial']))
                return 0
        else:
            print("System did not come back up. Not getting data on port: " + str(self.data_port))
        return 1

    def __del__(self):
        self.sensor.close()


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Kaarta Velodyne communication tool', allow_abbrev=False, conflict_handler="resolve")
    parser.add_argument('--ip_addr', type=str, nargs='?',
                        help='Specific IP address to connect to')
    parser.add_argument('--detect', action='store_true',
                        help='Use this to automatically determine the IP address of the system by listening on the default data port (self.data_port) for data')
    parser.add_argument('--setup_defaults_stencil_pro', action='store_true',
                        help='Setup the velodyne for standard Kaarta usage')
    parser.add_argument('--print_snapshot', action='store_true',
                        help='Setup the velodyne for standard Kaarta usage')
    parser.add_argument('--quiet', action='store_true',
                        help='Only output necessary data, such as detected ip when using --detect')
    parser.add_argument('--verbose', action='store_true',
                        help='Output extra debugging info')
    parser.add_argument('--print_serial', action='store_true', help='Print serial number', dest="print_serial")
    parser.add_argument('--print_model', action='store_true', help='Print model number', dest="print_model")
    parser.add_argument('--print_info', action='store_true', help='Print sensor info', dest="print_info")
    parser.add_argument('--print_diag', action='store_true', help='Print sensor diag', dest="print_diag")
    parser.add_argument('--print_snapshot', action='store_true', help='Print sensor snapshot', dest="print_snapshot")
    args = parser.parse_args()

    if (args.ip_addr is None):
        args.ip_addr = "auto"


    config = VelodyneConfigurator(args)

    if (args.detect):
        if (config.is_streaming_data):
            print(config.ip_addr)
            exit(0)
        else:
            exit(1)

    if (args.setup_defaults_stencil_pro):
        if (args.verbose):
            print("Configuring for stencil pro usage")
        exit(config.setup_stencil_pro_velodyne())

    if (args.print_info or args.print_serial or args.print_model):
        (response, status) = config.get_json('info.json')
        if response:
            if (args.print_info):
                pretty_print(status)
            elif(args.print_serial):
                print(status['serial'])
            elif(args.print_model):
                print(status['model'])
            exit(0)
        else:
            print("Failed to get info")
            exit(1)

    if (args.print_diag):
        (response, status) = config.get_json('diag.json')
        if response:
            pretty_print(status)
            exit(0)
        else:
            print("Failed to get info")
            exit(1)

    if (args.print_snapshot):
        (response, status) = config.get_json('snapshot.hdl')
        if response:
            pretty_print(status)
            exit(0)
        else:
            print("Failed to get info")
            exit(1)
