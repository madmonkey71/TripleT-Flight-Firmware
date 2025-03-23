#!/usr/bin/env python3
"""
Backup File Manager for TripleT Flight Firmware

This script helps manage backup files by:
1. Moving files with specific patterns to the backup directories
2. Creating timestamped backups of specified files
3. Ensuring backup files don't interfere with compilation
"""

import os
import re
import shutil
from datetime import datetime

# Backup directory paths
ROOT_BACKUP_DIR = "backup"
SRC_BACKUP_DIR = "src/backup"

# Ensure backup directories exist
os.makedirs(ROOT_BACKUP_DIR, exist_ok=True)
os.makedirs(SRC_BACKUP_DIR, exist_ok=True)

# Patterns for backup files
BACKUP_PATTERNS = [
    r".*_\d{8}_\d{6}\.cpp$",  # Files with timestamps like _20250323_132145.cpp
    r".*_backup\.cpp$",        # Files ending with _backup.cpp
    r"temp_.*\.cpp$",          # Files starting with temp_
    r".*\.cpp\.old$",          # Files ending with .cpp.old
]

def create_backup(filepath):
    """Create a timestamped backup of a file"""
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        return
    
    # Generate timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Determine base name and extension
    dirname, filename = os.path.split(filepath)
    basename, ext = os.path.splitext(filename)
    
    # Create new backup filename
    backup_name = f"{basename}_{timestamp}{ext}"
    
    # Determine backup directory
    backup_dir = SRC_BACKUP_DIR if dirname.startswith("src") or dirname == "src" else ROOT_BACKUP_DIR
    backup_path = os.path.join(backup_dir, backup_name)
    
    # Copy the file
    shutil.copy2(filepath, backup_path)
    print(f"Created backup: {backup_path}")
    return backup_path

def move_backup_files():
    """Move files matching backup patterns to backup directories"""
    # Check root directory
    for filename in os.listdir("."):
        if os.path.isfile(filename) and any(re.match(pattern, filename) for pattern in BACKUP_PATTERNS):
            dest = os.path.join(ROOT_BACKUP_DIR, filename)
            shutil.move(filename, dest)
            print(f"Moved {filename} to {dest}")
    
    # Check src directory
    for filename in os.listdir("src"):
        filepath = os.path.join("src", filename)
        if os.path.isfile(filepath) and any(re.match(pattern, filename) for pattern in BACKUP_PATTERNS):
            dest = os.path.join(SRC_BACKUP_DIR, filename)
            shutil.move(filepath, dest)
            print(f"Moved {filepath} to {dest}")

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Backup file manager")
    parser.add_argument("--backup", help="Create a backup of specified file")
    parser.add_argument("--clean", action="store_true", help="Move existing backup files to backup directories")
    
    args = parser.parse_args()
    
    if args.backup:
        create_backup(args.backup)
    
    if args.clean:
        move_backup_files()
    
    # If no args, show help
    if not (args.backup or args.clean):
        parser.print_help()

if __name__ == "__main__":
    main() 