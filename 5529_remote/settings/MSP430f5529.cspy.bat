@REM This batch file has been generated by the IAR Embedded Workbench
@REM C-SPY Debugger, as an aid to preparing a command line for running
@REM the cspybat command line utility using the appropriate settings.
@REM
@REM Note that this file is generated every time a new debug session
@REM is initialized, so you may want to move or rename the file before
@REM making changes.
@REM
@REM You can launch cspybat by typing the name of this batch file followed
@REM by the name of the debug file (usually an ELF/DWARF or UBROF file).
@REM
@REM Read about available command line parameters in the C-SPY Debugging
@REM Guide. Hints about additional command line parameters that may be
@REM useful in specific cases:
@REM   --download_only   Downloads a code image without starting a debug
@REM                     session afterwards.
@REM   --silent          Omits the sign-on message.
@REM   --timeout         Limits the maximum allowed execution time.
@REM 


"D:\Program Files (x86)\IAR Systems\Embedded Workbench 6.4 Evaluation\common\bin\cspybat" "D:\Program Files (x86)\IAR Systems\Embedded Workbench 6.4 Evaluation\430\bin\430proc.dll" "D:\Program Files (x86)\IAR Systems\Embedded Workbench 6.4 Evaluation\430\bin\430sim.dll"  %1 --plugin "D:\Program Files (x86)\IAR Systems\Embedded Workbench 6.4 Evaluation\430\bin\430bat.dll" --backend -B "--hardware_multiplier" "32" "--hwmult_type" "8" "-p" "D:\Program Files (x86)\IAR Systems\Embedded Workbench 6.4 Evaluation\430\config\MSP430F5529.ddf" "--core=430Xv2" "--data_model=small" "--iv_base" "0xFF80" "--no_wrap_around" "--cpu_bug_30" "--odd_word_check" "-d" "sim" "--derivativeSim" "MSP430F5529" 


