# see https://community.platformio.org/t/error-object-uses-vfp-register-arguments-firmware-elf-does-not/25263/2
# this script is needed cuz if you only define these in platformio.ini, they don't get applied to linker flags

Import("env")
flags = [
   "-mfloat-abi=hard",
   "-mfpu=fpv5-d16"
]
env.Append(CCFLAGS=flags, LINKFLAGS=flags)
