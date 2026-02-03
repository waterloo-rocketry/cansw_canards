# see https://community.platformio.org/t/error-object-uses-vfp-register-arguments-firmware-elf-does-not/25263/2
Import("env")
flags = [
   "-mfloat-abi=hard",
   "-mfpu=fpv4-sp-d16"
]
env.Append(CCFLAGS=flags, LINKFLAGS=flags)
