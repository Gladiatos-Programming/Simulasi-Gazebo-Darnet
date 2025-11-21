#!/usr/bin/env python3
# save as: remove_small_collisions.py

import re
import sys

# Daftar link kecil yang collision-nya mau dihapus
small_links = [
    "Hex_Spacer_MF_M3_L40mm",
    "FR05-S101",
    "FR05-F101",
    "HN05-i101",
    "MF128ZZ",
    "RX64-CAP",
    "2_FR07_H104",
    "10_FR07_H102",
    "14_FR07_S101",
    "FP04-F9",
    "6_PR13_B_SPACER_PELV",
    "SambunganServo_Revision",
]

def should_remove_collision(link_name):
    """Check if link is small part based on name"""
    for pattern in small_links:
        if pattern in link_name:
            return True
    return False

def remove_collisions_from_urdf(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Pattern untuk match link dengan collision
    link_pattern = r'<link name="([^"]+)">(.*?)</link>'
    
    def process_link(match):
        link_name = match.group(1)
        link_content = match.group(2)
        
        # Jika link kecil, hapus collision
        if should_remove_collision(link_name):
            # Hapus semua tag collision
            link_content_no_collision = re.sub(
                r'\s*<collision[^>]*>.*?</collision>\s*',
                '\n',
                link_content,
                flags=re.DOTALL
            )
            print(f"✓ Removed collision from: {link_name}")
            return f'<link name="{link_name}">{link_content_no_collision}</link>'
        else:
            return match.group(0)
    
    # Process semua link
    content_fixed = re.sub(link_pattern, process_link, content, flags=re.DOTALL)
    
    # Save
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content_fixed)
    
    print(f"\nDone! Output saved to: {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 remove_small_collisions.py input.xacro output.xacro")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    remove_collisions_from_urdf(input_file, output_file)