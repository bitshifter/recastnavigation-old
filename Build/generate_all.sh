#!/bin/sh

rm -rf Linux
rm -rf Xcode
rm -rf VC9
premake4 --to=Linux --os=linux gmake
premake4 --to=Xcode --os=macosx xcode3
premake4 --to=VC9 --os=windows vs2008
