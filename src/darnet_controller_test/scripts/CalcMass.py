#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import sys


def calculate_total_mass(urdf_file):
    """Calculate total mass dari URDF file"""
    
    try:
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        
        total_mass = 0.0
        link_masses = []
        
        # Cari semua link dengan inertial tag
        for link in root.findall('.//link'):
            link_name = link.get('name')
            inertial = link.find('inertial')
            
            if inertial is not None:
                mass_elem = inertial.find('mass')
                if mass_elem is not None:
                    mass_value = float(mass_elem.get('value', 0.0))
                    total_mass += mass_value
                    link_masses.append((link_name, mass_value))
        
        # Print hasil
        print("\n" + "="*60)
        print("ROBOT MASS CALCULATION")
        print("="*60)
        
        print(f"\nTotal Links with Mass: {len(link_masses)}")
        print(f"\nTotal Robot Mass: {total_mass:.4f} kg")
        print(f"                  {total_mass*1000:.2f} grams")
        
        # Print detail per link (sorted by mass)
        print("\n" + "-"*60)
        print("Mass per Link (heaviest first):")
        print("-"*60)
        
        link_masses.sort(key=lambda x: x[1], reverse=True)
        
        for link_name, mass in link_masses:  # Top 20 heaviest
            print(f"{link_name:40s}: {mass:8.4f} kg")
        
        if len(link_masses) > 20:
            print(f"... and {len(link_masses) - 20} more links")
        
        print("="*60 + "\n")
        
        return total_mass
        
    except Exception as e:
        print(f"Error: {e}")
        return None


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 calculate_robot_mass.py <urdf_file>")
        print("\nOr generate URDF first:")
        print("  ros2 run xacro xacro path/to/robot.xacro > /tmp/robot.urdf")
        print("  python3 calculate_robot_mass.py /tmp/robot.urdf")
        sys.exit(1)
    
    urdf_file = sys.argv[1]
    calculate_total_mass(urdf_file)