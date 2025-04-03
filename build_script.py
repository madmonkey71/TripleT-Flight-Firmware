Import("env")

# This script completely disables LTO for all environments
def before_build():
    # Disable LTO for all environments
    env.Replace(ARFLAGS="rcs")
    
    # Disable LTO for specific libraries
    env.Append(
        LIBLINKFLAGS=["-Wl,--plugin-opt=disable-lto"]
    )
    
    # Force disable LTO
    env.Append(
        CCFLAGS=["-fno-lto"],
        LINKFLAGS=["-fno-lto"]
    )
    
    # Print debug information
    print("ARFLAGS:", env.GetProjectOption("arflags"))
    print("LIBLINKFLAGS:", env.GetProjectOption("liblinkflags"))
    print("CCFLAGS:", env.GetProjectOption("ccflags"))
    print("LINKFLAGS:", env.GetProjectOption("linkflags"))

# Register the callback
env.AddPreAction("buildprog", before_build) 