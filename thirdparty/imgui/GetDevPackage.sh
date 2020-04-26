#!/usr/bin/sh
mkdir tmp
cd tmp
curl https://codeload.github.com/ocornut/imgui/zip/v1.76 -o v1.76.zip
unzip v1.76.zip

cd imgui-1.76

cp  ./*.cpp ../../imgui/
cp  ./*.h ../../imgui/
cp  ./examples/imgui_impl_opengl2.* ../../imgui/
cp  ./examples/imgui_impl_sdl.* ../../imgui/

cd ..
