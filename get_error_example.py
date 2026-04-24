#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Demonstrates how to use the GetError interface to get robot alarm information in real projects
"""

from dobot_api import DobotApiDashboard
import time
import json

class RobotErrorMonitor:
    """
    Robot Error Monitor Class 
    A class for monitoring robot alarm information
    """
    
    def __init__(self, robot_ip="192.168.200.1", dashboard_port=29999):
        self.robot_ip = robot_ip
        self.dashboard_port = dashboard_port
        self.dashboard = None
        
    def connect(self):
        """Connect to robot """
        try:
            self.dashboard = DobotApiDashboard(self.robot_ip, self.dashboard_port)
            print(f"Successfully connected to robot : {self.robot_ip}:{self.dashboard_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to robot : {e}")
            return False
    
    def disconnect(self):
        """Disconnect from robot """
        if self.dashboard:
            self.dashboard.close()
            print("Disconnected from robot ")
    
    def get_error_info(self, language="zh_cn"):
        """
        Get error information 
        
        Args:
                           "zh_cn" - Simplified Chinese 
                           "zh_hant" - Traditional Chinese   
                           "en" - English 
                           "ja" - Japanese 
                           "de" - German 
                           "vi" - Vietnamese 
                           "es" - Spanish 
                           "fr" - French 
                           "ko" - Korean 
                           "ru" - Russian 
        
        Returns:
            dict: Error information dictionary 
        """
        if not self.dashboard:
            print("Not connected to robot ")
            return None
            
        return self.dashboard.GetError(language)
    
    def check_errors(self, language="zh_cn"):
        """
        Check and display current error information 
        
        Args:
            language (str): Display language 
            
        Returns:
        """
        error_info = self.get_error_info(language)
        
        if not error_info or "errMsg" not in error_info:
            print("Failed to get error information ")
            return False
        
        errors = error_info["errMsg"]
        
        if not errors:
            return False
        
        print("=" * 50)
        
        for i, error in enumerate(errors, 1):
            print(f"Error  {i}:")
            print(f"  🆔 ID: {error.get('id', 'N/A')}")
            print(f"  📊 Level : {error.get('level', 'N/A')}")
            print(f"  📝 Description : {error.get('description', 'N/A')}")
            print(f"  🔧 Solution : {error.get('solution', 'N/A')}")
            print(f"  🏷️  Mode : {error.get('mode', 'N/A')}")
            print(f"  📅 Date : {error.get('date', 'N/A')}")
            print(f"  🕐 Time : {error.get('time', 'N/A')}")
            print("-" * 30)
        
        return True
    
    def monitor_errors(self, interval=5, language="zh_cn"):
        """
        Continuously monitor error information 
        
        Args:
            language (str): Display language 
        """
        
        try:
            while True:
                print(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] Checking error information ...")
                has_errors = self.check_errors(language)
                
                if has_errors:
                    print("\n⚠️  Recommend handling error information immediately ！")
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\nMonitoring stopped ")
    
    def save_error_log(self, filename=None, language="zh_cn"):
        """
        Save error information to file 
        
        Args:
            language (str): Language setting 
        """
        if filename is None:
            filename = f"robot_errors_{time.strftime('%Y%m%d_%H%M%S')}.json"
        
        error_info = self.get_error_info(language)
        
        if error_info:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(error_info, f, ensure_ascii=False, indent=2)
                print(f"Error information saved to : {filename}")
            except Exception as e:
                print(f"Failed to save file : {e}")
        else:
            print("Unable to get error information ")

def main():
    
    # Create monitor instance 
    monitor = RobotErrorMonitor()
    
    # Connect to robot 
    if not monitor.connect():
        return
    
    try:
        

        print("\n1. Check current error information :")
        monitor.check_errors("zh_cn")
        
        # 2. Multi-language support 
        print("\n2. Multi-language support demonstration :")
        languages = {
            "en": "English ",
        }
        
        for lang_code, lang_name in languages.items():
            print(f"\n--- {lang_name} ({lang_code}) ---")
            monitor.check_errors(lang_code)
        
        # 3. Save error log 
        print("\n3. Save error log :")
        monitor.save_error_log()
        
        # 4. Get raw data 
        raw_data = monitor.get_error_info("zh_cn")
        if raw_data:
            print(json.dumps(raw_data, ensure_ascii=False, indent=2))
        

        # print("\n5. Start continuous monitoring :")
        # monitor.monitor_errors(interval=10, language="zh_cn")
        
    finally:
        # Disconnect 
        monitor.disconnect()

if __name__ == "__main__":
    main()