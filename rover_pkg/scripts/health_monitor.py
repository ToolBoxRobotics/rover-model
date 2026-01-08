#!/usr/bin/env python3

import rospy
import psutil
import subprocess
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import BatteryState

class HealthMonitor:
    def __init__(self):
        rospy.init_node('health_monitor')
        self.pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        
        # Subscribe to Arduino Battery data
        self.bat_voltage = 0.0
        self.bat_current = 0.0
        rospy.Subscriber('rover/battery', BatteryState, self.bat_cb)
        
        # Timer for 1Hz updates
        rospy.Timer(rospy.Duration(1.0), self.update_diagnostics)

    def bat_cb(self, msg):
        self.bat_voltage = msg.voltage
        self.bat_current = msg.current

    def get_wifi_strength(self):
        # Reads signal level from standard linux file
        try:
            # cmd: iwconfig wlan0 | grep 'Signal level'
            output = subprocess.check_output("iwconfig wlan0 | grep 'Signal level'", shell=True).decode()
            if "Signal level=" in output:
                # Parse "-XX dBm"
                part = output.split("Signal level=")[1]
                dbm = int(part.split(" ")[0])
                return dbm
        except:
            return -100 # Error or no connection
        return -100

    def get_cpu_temp(self):
        try:
            temps = psutil.sensors_temperatures()
            if 'cpu_thermal' in temps:
                return temps['cpu_thermal'][0].current
            return 0.0
        except:
            return 0.0

    def update_diagnostics(self, event):
        arr = DiagnosticArray()
        arr.header.stamp = rospy.Time.now()

        # --- 1. Power System Status ---
        power_stat = DiagnosticStatus()
        power_stat.name = "Power System"
        power_stat.hardware_id = "INA219_Battery"
        
        # Logic to determine Level (OK, WARN, ERROR)
        if self.bat_voltage > 11.0:
            power_stat.level = DiagnosticStatus.OK
            power_stat.message = "Voltage Nominal"
        elif self.bat_voltage > 10.0:
            power_stat.level = DiagnosticStatus.WARN
            power_stat.message = "Low Battery"
        else:
            power_stat.level = DiagnosticStatus.ERROR
            power_stat.message = "Critical Voltage!"

        power_stat.values = [
            KeyValue("Voltage (V)", str(round(self.bat_voltage, 2))),
            KeyValue("Current (A)", str(round(self.bat_current, 2))),
            KeyValue("Power (W)", str(round(self.bat_voltage * self.bat_current, 2)))
        ]
        arr.status.append(power_stat)

        # --- 2. Compute System (RPi) ---
        comp_stat = DiagnosticStatus()
        comp_stat.name = "Raspberry Pi 4"
        comp_stat.hardware_id = "SBC_Main"
        
        cpu_temp = self.get_cpu_temp()
        cpu_usage = psutil.cpu_percent()
        ram = psutil.virtual_memory()
        
        if cpu_temp < 70:
            comp_stat.level = DiagnosticStatus.OK
            comp_stat.message = "Cool"
        elif cpu_temp < 80:
            comp_stat.level = DiagnosticStatus.WARN
            comp_stat.message = "Running Hot"
        else:
            comp_stat.level = DiagnosticStatus.ERROR
            comp_stat.message = "Overheating"

        comp_stat.values = [
            KeyValue("CPU Temp (C)", str(cpu_temp)),
            KeyValue("CPU Load (%)", str(cpu_usage)),
            KeyValue("RAM Used (%)", str(ram.percent))
        ]
        arr.status.append(comp_stat)

        # --- 3. Network Link ---
        net_stat = DiagnosticStatus()
        net_stat.name = "Wi-Fi Link"
        net_stat.hardware_id = "wlan0"
        
        rssi = self.get_wifi_strength()
        
        if rssi > -65:
            net_stat.level = DiagnosticStatus.OK
            net_stat.message = "Strong Signal"
        elif rssi > -80:
            net_stat.level = DiagnosticStatus.WARN
            net_stat.message = "Weak Signal"
        else:
            net_stat.level = DiagnosticStatus.ERROR
            net_stat.message = "Link Critical/Lost"

        net_stat.values = [KeyValue("RSSI (dBm)", str(rssi))]
        arr.status.append(net_stat)

        self.pub.publish(arr)

if __name__ == '__main__':
    HealthMonitor()
    rospy.spin()
