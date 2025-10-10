#!/usr/bin/env python3
import trimesh
import numpy as np
import math
import os

def calculate_box_inertia(length, width, height, mass):
    """Calculate inertia tensor for a box (L x W x H)"""
    Ixx = (1/12) * mass * (width**2 + height**2)
    Iyy = (1/12) * mass * (length**2 + height**2)
    Izz = (1/12) * mass * (length**2 + width**2)
    
    return np.array([
        [Ixx, 0.0, 0.0],
        [0.0, Iyy, 0.0],
        [0.0, 0.0, Izz]
    ]), np.array([0.0, 0.0, 0.0])

def calculate_cylinder_inertia(radius, height, mass):
    """Calculate inertia tensor for a cylinder"""
    Ixx = Iyy = (1/12) * mass * (3 * radius**2 + height**2)
    Izz = (1/2) * mass * radius**2
    
    return np.array([
        [Ixx, 0.0, 0.0],
        [0.0, Iyy, 0.0],
        [0.0, 0.0, Izz]
    ]), np.array([0.0, 0.0, 0.0])

def calculate_sphere_inertia(radius, mass):
    """Calculate inertia tensor for a sphere"""
    I = (2/5) * mass * radius**2
    
    return np.array([
        [I, 0.0, 0.0],
        [0.0, I, 0.0],
        [0.0, 0.0, I]
    ]), np.array([0.0, 0.0, 0.0])

def calculate_from_stl(stl_file, mass, scale):
    """Calculate inertia from STL mesh file"""
    if not os.path.exists(stl_file):
        raise FileNotFoundError(f"STL file not found: {stl_file}")
    
    # Load mesh
    mesh = trimesh.load(stl_file)
    
    # Scale the mesh
    mesh.apply_scale(scale)
    
    # Calculate mass properties
    props_density1 = mesh.mass_properties
    
    # Get inertia matrix at density=1
    inertia_density1 = props_density1['inertia']
    volume = props_density1['volume']
    
    # Calculate required density for desired mass
    required_density = mass / volume
    
    # Scale inertia matrix
    inertia_scaled = inertia_density1 * required_density
    
    # Get center of mass
    com = props_density1['center_mass']
    
    return inertia_scaled, com

def print_results(mass, com, inertia_matrix, shape_info=""):
    """Print the results in both URDF and SDF format"""
    print(f"\n{'='*50}")
    print(f"RESULTS {shape_info}")
    print(f"{'='*50}")
    print(f"Mass: {mass} kg")
    print(f"Center of Mass: {com[0]:.6f} {com[1]:.6f} {com[2]:.6f}")
    
    print(f"\n--- URDF FORMAT ---")
    print(f"<inertial>")
    print(f"  <origin xyz=\"{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}\" rpy=\"0 0 0\"/>")
    print(f"  <mass value=\"{mass}\"/>")
    print(f"  <inertia ixx=\"{inertia_matrix[0,0]:.6e}\" iyy=\"{inertia_matrix[1,1]:.6e}\" izz=\"{inertia_matrix[2,2]:.6e}\"")
    print(f"           ixy=\"{inertia_matrix[0,1]:.6e}\" iyz=\"{inertia_matrix[1,2]:.6e}\" ixz=\"{inertia_matrix[0,2]:.6e}\"/>")
    print(f"</inertial>")
    
    print(f"\n--- SDF FORMAT ---")
    print(f"<inertial>")
    print(f"  <pose>{com[0]:.6f} {com[1]:.6f} {com[2]:.6f} 0 0 0</pose>")
    print(f"  <mass>{mass}</mass>")
    print(f"  <inertia>")
    print(f"    <ixx>{inertia_matrix[0,0]:.6f}</ixx>")
    print(f"    <iyy>{inertia_matrix[1,1]:.6f}</iyy>")
    print(f"    <izz>{inertia_matrix[2,2]:.6f}</izz>")
    print(f"    <ixy>{inertia_matrix[0,1]:.6f}</ixy>")
    print(f"    <ixz>{inertia_matrix[0,2]:.6f}</ixz>")
    print(f"    <iyz>{inertia_matrix[1,2]:.6f}</iyz>")
    print(f"  </inertia>")
    print(f"</inertial>")

def get_float_input(prompt, default=None):
    """Get float input with optional default value"""
    while True:
        try:
            if default is not None:
                user_input = input(f"{prompt} (default: {default}): ").strip()
                if not user_input:
                    return default
            else:
                user_input = input(f"{prompt}: ").strip()
            
            return float(user_input)
        except ValueError:
            print("Please enter a valid number!")

def get_file_input(prompt):
    """Get file path input with validation"""
    while True:
        file_path = input(f"{prompt}: ").strip()
        if os.path.exists(file_path):
            return file_path
        else:
            print(f"File not found: {file_path}")
            print("Please enter a valid file path.")

def main():
    print("="*60)
    print("        INERTIA CALCULATOR")
    print("="*60)
    print("Calculate inertia properties for robot parts")
    print()
    
    while True:
        print("Choose calculation method:")
        print("1. STL file (complex meshes)")
        print("2. Box shape (length × width × height)")
        print("3. Cylinder shape (radius × height)")
        print("4. Sphere shape (radius)")
        print("5. Exit")
        
        choice = input("\nEnter your choice (1-5): ").strip()
        
        try:
            if choice == '1':
                print("\n--- STL FILE CALCULATION ---")
                stl_file = get_file_input("Enter STL file path")
                mass = get_float_input("Enter mass (kg)", 0.1)
                scale = get_float_input("Enter scale factor (mm->m use 0.001)", 0.001)
                
                inertia_matrix, com = calculate_from_stl(stl_file, mass, scale)
                shape_info = f"(STL: {os.path.basename(stl_file)})"
                
            elif choice == '2':
                print("\n--- BOX CALCULATION ---")
                length = get_float_input("Enter length (X dimension) in meters")
                width = get_float_input("Enter width (Y dimension) in meters")
                height = get_float_input("Enter height (Z dimension) in meters")
                mass = get_float_input("Enter mass (kg)")
                
                inertia_matrix, com = calculate_box_inertia(length, width, height, mass)
                shape_info = f"(Box: {length}×{width}×{height}m)"
                
            elif choice == '3':
                print("\n--- CYLINDER CALCULATION ---")
                radius = get_float_input("Enter radius in meters")
                height = get_float_input("Enter height in meters")
                mass = get_float_input("Enter mass (kg)")
                
                inertia_matrix, com = calculate_cylinder_inertia(radius, height, mass)
                shape_info = f"(Cylinder: r={radius}m, h={height}m)"
                
            elif choice == '4':
                print("\n--- SPHERE CALCULATION ---")
                radius = get_float_input("Enter radius in meters")
                mass = get_float_input("Enter mass (kg)")
                
                inertia_matrix, com = calculate_sphere_inertia(radius, mass)
                shape_info = f"(Sphere: r={radius}m)"
                
            elif choice == '5':
                print("Goodbye!")
                break
                
            else:
                print("Invalid choice! Please enter 1-5.")
                continue
            
            print_results(mass, com, inertia_matrix, shape_info)
            
            # Ask if user wants to continue
            print("\n" + "="*50)
            continue_choice = input("Calculate another part? (y/n): ").strip().lower()
            if continue_choice not in ['y', 'yes']:
                print("Goodbye!")
                break
                
        except Exception as e:
            print(f"Error: {e}")
            print("Please try again.")

if __name__ == '__main__':
    main()