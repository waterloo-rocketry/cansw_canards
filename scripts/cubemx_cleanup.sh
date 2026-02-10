DST=src/third_party/cubemx_autogen

mkdir -p "$DST/src"

# rm old autogen folders
rm -rf "$DST/Drivers"
rm -rf "$DST/Middlewares"
rm -rf "$DST/Startup"

# move top-level files from src/ to dst/src (none of our source code is top-level, so this is safe)
find "src" -maxdepth 1 -type f -exec mv {} "$DST/src/" \;
# move auto-gen folders from src/ to dst/
mv -f Drivers $DST
mv -f Middlewares $DST
mv -f Startup $DST

# we'll never need these files
rm -rf ThreadSafe
rm -f .cproject
rm -f .project
rm -f STM32*_RAM.ld
