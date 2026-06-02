cd ../closedrocket
git checkout codegen
git pull
cd ../cansw_canards
cp -r ../closedrocket/embedded-coder/codegen/lib/GNC_codegen src/third_party
rm -rf src/third_party/GNC_codegen/examples
rm -rf src/third_party/GNC_codegen/html
rm -rf src/third_party/GNC_codegen/interface