import os
import shutil
import sys
import subprocess
import re
from pathlib import Path
from collections import defaultdict
from datetime import datetime

def calculate_td2(target_dir):
    """Calculates TD2 from IMU files."""
    imu_files = sorted([f for f in target_dir.glob("imu*.csv") if f.is_file()])
    
    print(f"[DEBUG] Searching for IMU files in: {target_dir}")
    print(f"[DEBUG] IMU files found: {[f.name for f in imu_files]}")
    
    if not imu_files:
        print(f"[DEBUG] No imu*.csv files found in {target_dir}")
        return 0.0
    
    def extract_first_number(line):
        """Extracts the first number from line, regardless of separator."""
        match = re.search(r'^(\d+)', line.strip())
        if match:
            return int(match.group(1))
        return None
    
    try:
        # First file
        print(f"[DEBUG] Analyzing first file: {imu_files[0].name}")
        with imu_files[0].open("r") as f:
            first_line = f.readline().strip()
            print(f"[DEBUG] First line: {first_line}")
            if not first_line:
                print(f"[DEBUG] First line is empty")
                return 0.0
            Ta = extract_first_number(first_line)
            if Ta is None:
                print(f"[DEBUG] No number found in first line")
                return 0.0
            print(f"[DEBUG] Ta = {Ta}")
        
        # Last file
        print(f"[DEBUG] Analyzing last file: {imu_files[-1].name}")
        with imu_files[-1].open("r") as f:
            lines = f.readlines()
            if not lines:
                print(f"[DEBUG] Last file is empty")
                return 0.0
            last_line = lines[-1].strip()
            print(f"[DEBUG] Last line: {last_line}")
            if not last_line:
                print(f"[DEBUG] Last line is empty")
                return 0.0
            Tb = extract_first_number(last_line)
            if Tb is None:
                print(f"[DEBUG] No number found in last line")
                return 0.0
            print(f"[DEBUG] Tb = {Tb}")
            
        td2 = (Tb - Ta) / 1e9
        print(f"[DEBUG] TD2 calculated: {td2:.6f} seconds")
        return td2
        
    except Exception as e:
        print(f"[WARN] Error calculating TD2 in {target_dir.name}: {e}")
        print(f"[DEBUG] Error details: {type(e).__name__}: {str(e)}")
    
    return 0.0

# ===== Default configurations =====
def_config = {
    'mod': None,
    'start_index': 5,
    'end_index': 100,
    'pas_index': 5,
    'pas': 15,
    'suprapunere': 1,
    'index_end_g3': 25,  # For g3 mode
    'input_dir': Path('.'),
    'output_dir': Path('.'),
    # Working directory - if empty "", current directory is used
    'working_dir': "e:\\@HDMapping MANDEYE\\continousScanning_0042_Tismana1_2025-06-27\\"
}

args = sys.argv[1:]
config = def_config.copy()
created_dirs = []

# ===== Working directory setup =====
if config['working_dir'] and config['working_dir'].strip():
    os.chdir(config['working_dir'])
    # Working directory setup
    print(f"[INFO] Working directory set to: {config['working_dir']}")
else:
    print(f"[INFO] Using current directory: {os.getcwd()}")

# ===== STAGE 1: only if arguments exist =====
if len(args) > 0:
    if args[0] == 'h' or args[0] == 'help':
        print("""
=== Data Set Creation Tool for Processing Tests - Help ===

This tool provides two main processing stages for organizing and processing lidar scanning data:

STAGE 1 - Data Set Generation (with arguments):
----------------------------------------
Generates organized data sets from raw scan files based on numerical indices found in filenames.
Creates test data sets for processing validation and analysis.
Supported file extensions: .gnss, .nmea, .csv, .laz, .sn, .json

MODE g1 - Progressive Sets:
    Usage: python pas1.py g1 [start_index] [end_index] [step]
    Creates progressive sets from index 0 to various endpoints.
    Default: start=5, end=100, step=5
    Example: g1 10 50 10 → creates set_0000-0010, set_0000-0020, set_0000-0030, etc.

MODE g2 - Sliding Window Sets:
    Usage: python pas1.py g2 [window_size] [overlap]
    Creates overlapping sets using a sliding window approach.
    Default: window_size=15, overlap=1
    Example: g2 5 1 → creates overlapping sets of 5 files with 1 file overlap

MODE g3 - Multiple Target Sets:
    Usage: python pas1.py g3 [target1] [target2] [target3] ...
    Creates multiple sets from index 0 to each specified target.
    Default: target=25
    Example: g3 15 25 35 → creates set_0000-0015, set_0000-0025, set_0000-0035
    Note: Processing stops if any target exceeds available data range.

Features:
- Automatically detects available file indices and limits processing to available data
- Set names reflect actual available indices in directory structure
- Skips existing directories to avoid overwriting
- Maximizes data utilization within available file range

STAGE 2 - Lidar Processing (with specific arguments):
----------------------------------------
Processes directories using the lidar odometry application.

MODE set - Process Set Directories:
    Usage: python pas1.py set
    Processes only set_XXXX-YYYY directories in current working directory.
    If no set directories found, nothing is processed.

MODE all - Process All Valid Directories:
    Usage: python pas1.py all
    Processes all directories containing files with supported extensions and valid indices.
    Searches for directories with .gnss, .nmea, .csv, .laz, .sn, .json files regardless of directory name.

Features:
- Generates detailed processing logs in procesare_log.tsv
- Calculates field measurement duration (TD2) from IMU data
- Creates timestamped subdirectories with processing output
- Supports log continuation across multiple runs

Log Format:
Folder | Start_date | Start_time | End_date | End_time | Process(s) | Field(s) | P/F(%)

Configuration (hardcoded in script):
- Working Directory: Set in 'working_dir' variable (if empty, uses current directory)
- Processing Executable: lidar_odometry_step_1_084-v3.exe (path configured in script)
- Parameters File: .toml configuration file (path configured in script)
- Output Folder Name: Dynamic format 'res_YY-MM-DD-HH-MM_F[td2]_[td1]' where:
  * res_ = result prefix
  * YY-MM-DD-HH-MM = processing start timestamp
  * F[td2] = field measurement duration in seconds (from IMU data)
  * [td1] = processing time in seconds

Examples:
    python pas1.py h            # Show this help
    python pas1.py g1 5 30 5    # Generate progressive sets
    python pas1.py g2 10 2      # Generate sliding window sets  
    python pas1.py g3 20 40 60  # Generate multiple target sets
    python pas1.py set          # Process only set_XXXX-YYYY directories
    python pas1.py all          # Process all directories with valid data files

Note: Stage 1 exits after set generation. Stage 2 processes data based on specified mode.
All paths (working directory, executable, parameters file) are configured within the script code.
""")
        sys.exit(0)
    elif args[0] in ('g1', 'g2', 'g3'):
        config['mod'] = args[0]

        if config['mod'] == 'g1':
            if len(args) >= 2:
                config['start_index'] = int(args[1])
            if len(args) >= 3:
                config['end_index'] = int(args[2])
            if len(args) >= 4:
                config['pas_index'] = int(args[3])

        elif config['mod'] == 'g2':
            if len(args) >= 2:
                config['pas'] = int(args[1])
            if len(args) >= 3:
                config['suprapunere'] = int(args[2])

        elif config['mod'] == 'g3':
            # For g3, each argument becomes an index_end for a separate set
            indices_to_process = []
            if len(args) >= 2:
                # First argument
                indices_to_process.append(int(args[1]))
                # Additional arguments
                for i in range(2, len(args)):
                    try:
                        indices_to_process.append(int(args[i]))
                    except ValueError:
                        print(f"[WARN] Argument {args[i]} is not a valid number. Ignoring.")
            else:
                # If no arguments, use default value
                indices_to_process.append(config['index_end_g3'])

        try:
            all_files = os.listdir(config['input_dir'])
            index_map = defaultdict(list)
            print("[DEBUG] Analyzing files...")

            for f in all_files:
                for ext in ['.gnss', '.nmea', '.csv', '.laz', '.sn', '.json']:
                    if f.endswith(ext):
                        filename_no_ext = f[:-len(ext)]
                        digits = ''.join(filter(str.isdigit, filename_no_ext))
                        if len(digits) >= 4:
                            index = int(digits[-4:])
                            index_map[index].append(f)
                            print(f"  [+] Accepted: {f} -> index {index:04d}")
                        else:
                            print(f"  [-] Ignored (no valid index): {f}")

            sorted_indices = sorted(index_map.keys())
            if not sorted_indices:
                raise ValueError("No valid numerical indices found.")

            def copy_set_files(index_range, set_name):
                dest_dir = config['output_dir'] / set_name
                if dest_dir.exists():
                    print(f"[SKIP] {set_name} already exists. Skipping.")
                    return
                dest_dir.mkdir(parents=True, exist_ok=True)
                created_dirs.append(dest_dir)
                print(f"  - {set_name} (indices: {index_range.start:04d} - {index_range.stop - 1:04d})")
                for idx in index_range:
                    for f in index_map.get(idx, []):
                        src = config['input_dir'] / f
                        dst = dest_dir / f
                        shutil.copy2(src, dst)

            # Determine available limits
            min_available = min(sorted_indices) if sorted_indices else 0
            max_available = max(sorted_indices) if sorted_indices else 0
            print(f"[INFO] Available indices: {min_available:04d} - {max_available:04d}")

            if config['mod'] == 'g1':
                start = config['start_index']
                end = min(config['end_index'], max_available)  # Limit to maximum available
                pas = config['pas_index']
                j = start
                while j <= end:
                    # Determine real indices for this set
                    actual_indices = [idx for idx in range(0, j + 1) if idx in sorted_indices]
                    if actual_indices:
                        actual_min = min(actual_indices)
                        actual_max = max(actual_indices)
                        set_name = f"set_{actual_min:04d}-{actual_max:04d}"
                        print(f"[INFO] Creating set: {set_name} (available indices: {actual_min:04d} - {actual_max:04d})")
                        copy_set_files(range(0, j + 1), set_name)
                    j += pas

            elif config['mod'] == 'g2':
                pas = config['pas']
                supr = config['suprapunere']
                max_idx = max_available  # Use maximum available
                i = min_available  # Start from minimum available
                while i <= max_idx:
                    j = min(i + pas - 1, max_idx)
                    # Determine real indices for this set
                    actual_indices = [idx for idx in range(i, j + 1) if idx in sorted_indices]
                    if actual_indices:
                        actual_min = min(actual_indices)
                        actual_max = max(actual_indices)
                        set_name = f"set_{actual_min:04d}-{actual_max:04d}"
                        print(f"[INFO] Creating set: {set_name} (available indices: {actual_min:04d} - {actual_max:04d})")
                        copy_set_files(range(i, j + 1), set_name)
                    i = j - supr + 1

            elif config['mod'] == 'g3':
                # Process each index from the list
                for index_end in indices_to_process:
                    if index_end > max_available:
                        print(f"[WARN] Index {index_end:04d} exceeds maximum available ({max_available:04d}). Stopping set creation.")
                        break
                    
                    # Determine real indices for this set
                    actual_indices = [idx for idx in range(0, index_end + 1) if idx in sorted_indices]
                    if actual_indices:
                        actual_min = min(actual_indices)
                        actual_max = max(actual_indices)
                        set_name = f"set_{actual_min:04d}-{actual_max:04d}"
                        print(f"[INFO] Creating set: {set_name} (available indices: {actual_min:04d} - {actual_max:04d})")
                        copy_set_files(range(0, index_end + 1), set_name)
                    else:
                        print(f"[WARN] No available indices for range 0000-{index_end:04d}")

            print("\n[INFO] Sets have been generated in current directory.")
            print("[INFO] Stage 1 completed. Process terminated.")
            sys.exit(0)
        except Exception as e:
            print(f"[ERROR] Stage 1 failed: {e}")
            sys.exit(1)
    elif args[0] in ('set', 'all'):
        # STAGE 2 with specific arguments
        process_mode = args[0]
    else:
        # Argument unknown - show help
        print(f"[ERROR] Unknown argument: {args[0]}")
        print("For help, run: python pas1.py h")
        sys.exit(1)
else:
    # No arguments - show configuration and simplified help
    print("""
Data Set Creation Tool for Processing Tests
""")
    
    # Display current configuration
    print(f"[INFO] Working directory: {os.getcwd()}")
    print("[INFO] This configuration determines where directories are searched for processing")
    
    exe_path = Path(r"d:\^# instrumente topo\_MANDEYE\083\lidar_odometry_step_1_084-v3.exe")
    config_file = Path(r"d:\^# instrumente topo\_MANDEYE\083\fast_7a incet 080.toml")
    timestamp_base = datetime.now().strftime('%y-%m-%d-%H-%M')
    output_folder_template = f"res_{timestamp_base}_"
    
    print(f"[INFO] EXE Application: {exe_path.name}")
    print(f"[INFO] Parameters file: {config_file.name}")
    print(f"[INFO] Output directory: {output_folder_template}F[td2]_[td1] (td2=field, td1=processing, calculated after execution)")
    
    print("""
Usage examples:
    python pas1.py set    # Process only set_XXXX-YYYY directories
    python pas1.py all    # Process all directories with valid data files
    python pas1.py h      # Show detailed help with all options

For complete documentation and data set generation options, use: python pas1.py h
""")
    sys.exit(0)

# ===== STAGE 2 =====
def has_valid_files(directory):
    """Checks if a directory contains files with supported extensions and valid indices."""
    supported_extensions = ['.gnss', '.nmea', '.csv', '.laz', '.sn', '.json']
    for file in directory.iterdir():
        if file.is_file():
            for ext in supported_extensions:
                if file.name.endswith(ext):
                    filename_no_ext = file.name[:-len(ext)]
                    digits = ''.join(filter(str.isdigit, filename_no_ext))
                    if len(digits) >= 4:
                        return True
    return False

print(f"[INFO] Stage 2 execution in mode: {process_mode}")

# Display current configuration
print(f"[INFO] Working directory: {os.getcwd()}")
print("[INFO] This configuration determines where directories are searched for processing")

exe_path = Path(r"d:\^# instrumente topo\_MANDEYE\083\lidar_odometry_step_1_084-v3.exe")
config_file = Path(r"d:\^# instrumente topo\_MANDEYE\083\fast_7a incet 080.toml")
log_file = Path("procesare_log.tsv")
timestamp_base = datetime.now().strftime('%y-%m-%d-%H-%M')
output_folder_template = f"res_{timestamp_base}_"

print(f"[INFO] EXE Application: {exe_path.name}")
print(f"[INFO] Parameters file: {config_file.name}")
print(f"[INFO] Output directory: {output_folder_template}F[td2]_[td1] (td2=field, td1=processing, calculated after execution)")

if exe_path.exists():
    print("\n[INFO] Launching application for each set...")
    
    # Check if log exists and has header
    write_header = not log_file.exists()
    if log_file.exists():
        with log_file.open("r", encoding="utf-8") as f:
            first_line = f.readline().strip()
            if not first_line.startswith("Folder"):
                write_header = True
    
    # If log exists, no longer check processed sets - reprocess everything
    processed_sets = set()
    
    with log_file.open("a", encoding="utf-8", newline='') as log:
        if write_header:
            # Write EXE arguments first for new log
            log.write("="*50 + "\n")
            log.write(f"{exe_path.name}\n")
            log.write(f"{config_file.name}\n")
            log.write("="*50 + "\n")
            log.write("Folder	Start_date	Start_time	End_date	End_time	Process(s)	Field(s)	P/F(%)\n")
            print(f"[INFO] Header written to log: {log_file}")
        else:
            print(f"[INFO] Continuing existing log: {log_file}")
            # Check if last line ends with newline
            log_file_size = log_file.stat().st_size
            if log_file_size > 0:
                with log_file.open("rb") as check_file:
                    check_file.seek(-1, 2)  # Go to last character
                    last_char = check_file.read(1)
                    if last_char != b'\n':
                        log.write('\n')  # Add newline if missing
            
            # Write information about new processing session
            log.write("="*50 + "\n")
            log.write(f"{exe_path.name}\n")
            log.write(f"{config_file.name}\n")
            log.flush()
        
        t0 = datetime.now()
        
        # Dynamically scan directories at each iteration
        while True:
            if process_mode == 'set':
                # Find only set_XXXX-YYYY directories
                current_sets = [d for d in Path('.').iterdir()
                              if d.is_dir() and d.name.startswith("set_") and '-' in d.name]
                pending_sets = current_sets
                
                if not current_sets:
                    print("[INFO] No set_XXXX-YYYY directories found for processing.")
                    break
                    
            elif process_mode == 'all':
                # Find all directories with valid files
                all_dirs = [d for d in Path('.').iterdir() if d.is_dir()]
                pending_sets = [d for d in all_dirs if has_valid_files(d)]
                
                if not pending_sets:
                    print("[INFO] No directories with valid files found for processing.")
                    break
            
            if not pending_sets:
                print("[INFO] No more sets to process")
                break
                
            print(f"[INFO] Processing {len(pending_sets)} directories")
            
            for target_dir in sorted(pending_sets, key=lambda x: x.name):
                # Determine name for log
                dir_name = target_dir.name
                print(f"[INFO] Processing: {dir_name}")
                
                # Check again if directory still exists
                if not target_dir.exists():
                    print(f"[WARN] Directory {dir_name} was deleted. Skipping.")
                    continue
                    
                t_start = datetime.now()
                log.write(f"{dir_name}	{t_start.strftime('%a %m/%d/%Y')}	{t_start.strftime('%H:%M:%S.%f')[:-3]}	")
                log.flush()

                # Create initial directory with partial name
                initial_folder_name = f"{output_folder_template}"
                initial_output_subdir = target_dir / initial_folder_name
                initial_output_subdir.mkdir(exist_ok=True)

                print(f"[INFO] Initial output directory created: {initial_folder_name}")
                print(f"  > Execute: {exe_path.name} {target_dir} {config_file} {initial_output_subdir}")
                subprocess.run([str(exe_path), str(target_dir), str(config_file), str(initial_output_subdir)])

                t_end = datetime.now()
                td1 = (t_end - t_start).total_seconds()  # Processing time (td1)
                td1_int = int(td1)  # Convert to integer (seconds without decimals)

                # Calculate TD2 to finalize directory name
                td2 = calculate_td2(target_dir)
                td2_int = int(td2)  # Convert to integer (seconds without decimals)
                
                # Final directory name: res_YY-MM-DD-HH-MM_F[td2]_[td1]
                final_folder_name = f"{output_folder_template}F{td2_int}_{td1_int}"
                final_output_subdir = target_dir / final_folder_name
                
                # Rename directory to final name
                if initial_output_subdir.exists() and not final_output_subdir.exists():
                    initial_output_subdir.rename(final_output_subdir)
                    print(f"[INFO] Directory renamed to: {final_folder_name}")
                
                print(f"[INFO] TD1 (processing): {td1:.2f} seconds")
                print(f"[INFO] TD2 (field): {td2:.2f} seconds")

                # Calculate P/F ratio (%)
                if td2 > 0:
                    pf_ratio = (td1 / td2) * 100
                else:
                    pf_ratio = 0.0

                log.write(f"{t_end.strftime('%a %m/%d/%Y')}	{t_end.strftime('%H:%M:%S.%f')[:-3]}	{td1:.2f}	{td2:.2f}	{pf_ratio:.2f}%\n")
                log.flush()
            
            # No longer check for new sets - process all available sets once
            break

    print(f"\n[INFO] Log saved: {log_file}")
else:
    print("[WARNING] EXE application not found. Stage 2 skipped.")
