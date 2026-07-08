#!/usr/bin/env python3
import os
import glob
import yaml

def main():
    # Paths assuming we are in Ebrahim_Master_Thesis_repo/f1tenth_simulator/scripts
    # or running from workspace root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(os.path.dirname(script_dir))
    
    # Alternatively if someone runs this from somewhere else, we can fallback
    racetracks_dir = os.path.join(repo_root, 'f1tenth_racetracks')
    config_file = os.path.join(repo_root, 'f1tenth_simulator', 'config', 'sim.yaml')

    if not os.path.exists(racetracks_dir):
        print(f"Error: Could not find racetracks directory at {racetracks_dir}")
        return

    if not os.path.exists(config_file):
        print(f"Error: Could not find config file at {config_file}")
        return

    print(f"Scanning for tracks in {racetracks_dir}...")
    
    available_tracks = []
    
    # Iterate over all items in the racetracks directory
    for item in os.listdir(racetracks_dir):
        track_dir = os.path.join(racetracks_dir, item)
        if os.path.isdir(track_dir):
            # Check if map yaml exists
            map_yaml = os.path.join(track_dir, f"{item}_map.yaml")
            if os.path.exists(map_yaml):
                available_tracks.append(item)

    available_tracks.sort()
    print(f"Found {len(available_tracks)} available tracks: {available_tracks}")

    # Load sim.yaml
    with open(config_file, 'r') as f:
        config_data = yaml.safe_load(f)

    # Update available_tracks
    try:
        config_data['/**']['ros__parameters']['available_tracks'] = available_tracks
    except KeyError:
        print("Error: sim.yaml does not have the expected structure (/** -> ros__parameters).")
        return

    # Write back sim.yaml
    with open(config_file, 'w') as f:
        yaml.dump(config_data, f, default_flow_style=False, sort_keys=False)

    print(f"Successfully updated available_tracks in {config_file}")

if __name__ == '__main__':
    main()
