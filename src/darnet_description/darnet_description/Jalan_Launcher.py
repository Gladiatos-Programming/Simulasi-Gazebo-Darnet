import subprocess
import time

def main():
    print("Menjalankan robot...")

    try:
        subprocess.run(["ros2", "run", "darnet_description", "Jalan"], check=True)
        subprocess.run(["ros2", "run", "darnet_description", "SineWaveTarget"], check=True)
    except KeyboardInterrupt:
        print("\nProgram dihentikan.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()