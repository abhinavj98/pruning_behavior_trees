import subprocess
import argparse
import os

def start_rosbag_recording(filename):
    # Launch the ros2 bag record process with the custom file name
    process = subprocess.Popen(
        ['ros2', 'bag', 'record', '-a', '-o', filename], 
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    print(f"Started recording to rosbag: {filename}.")
    return process

def stop_rosbag_recording(process):
    # Stop the rosbag recording process
    process.terminate()
    try:
        # Wait for the process to terminate gracefully
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()  # Forcefully kill if still running after the timeout
        print("Process killed after timeout.")
    print("Stopped recording.")

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Record all ROS2 topics into a custom rosbag file.")
    parser.add_argument('--tree_type', type=str, required=True, help="Type of the tree (e.g., 'pruner', 'test').")
    parser.add_argument('--trial_number', type=int, required=True, help="Trial number (e.g., 1, 2, 3).")
    parser.add_argument('--tree_number', type=int, required=True, help="Tree number (e.g., 1, 2, 3).")

    # Parse the arguments
    args = parser.parse_args()

    # Construct filename based on provided arguments
    filename = f"{args.tree_type}_{args.tree_number}_{args.trial_number}"

    # Make sure the output directory exists
    os.makedirs("rosbags", exist_ok=True)  # Create a folder 'rosbags' to store the bag files
    
    # Start recording the rosbag
    process = start_rosbag_recording(f"rosbags/{filename}")

    try:
        # Wait for the user to stop recording (or any other mechanism to terminate)
        input("Press Enter to stop recording...")
    finally:
        # Stop the recording after user presses Enter or any other mechanism to stop
        stop_rosbag_recording(process)

if __name__ == "__main__":
    main()
