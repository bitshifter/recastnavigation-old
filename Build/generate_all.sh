#!/bin/sh

premake4 clean
premake4 --os=linux gmake
premake4 --os=macosx xcode3
premake4 --os=windows vs2008
