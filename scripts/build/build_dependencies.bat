@echo off

:try_vc17
rem Try configure for VC17
set VCVARS_FILE="C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
if not exist %VCVARS_FILE% goto try_vc16
call %VCVARS_FILE%
goto build_dependencies

:try_vc16
rem Try configure for VC16
set VCVARS_FILE="C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvars64.bat"
if not exist %VCVARS_FILE% goto unknown_compiler
call %VCVARS_FILE%
goto build_dependencies

rem Uknown compiler (different version of MSVC or another brand)
:unknown_compiler
echo "Unknown compiler."
goto build_dependencies

:build_dependencies
python scripts\build\build_dependencies.py %*
goto end

:end
