RET_TO=src

ORI=src/third_party/cubemx_autogen

find "$ORI/src" -type f -exec cp {} "$RET_TO/" \;

cp -r "$ORI/Drivers" "$RET_TO/.."
cp -r "$ORI/Middlewares" "$RET_TO/.."
cp -r "$ORI/Startup" "$RET_TO/.."


