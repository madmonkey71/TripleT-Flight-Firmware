Import("env")

# Disable LTO for all environments
env.Replace(ARFLAGS="rcs")

# Disable LTO for specific libraries
env.Append(
    LIBLINKFLAGS=["-Wl,--plugin-opt=disable-lto"]
)

# Print debug information
print("ARFLAGS:", env.GetProjectOption("arflags"))
print("LIBLINKFLAGS:", env.GetProjectOption("liblinkflags")) 