Import("env")

# This script patches the build process to bypass LTO issues
def patch_build_process(source, target, env):
    # Get the original command
    cmd = env.get("ARCOM")
    
    # Replace the command with a version that doesn't use LTO
    env.Replace(ARCOM="arm-none-eabi-ar rcs $TARGET $SOURCES")
    
    # Print debug information
    print("Patched ARCOM:", env.get("ARCOM"))

# Register the callback
env.AddPreAction("$ARCOM", patch_build_process) 