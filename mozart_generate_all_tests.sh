# MESI
make clean
IS_ESP=1 COH_MODE=0 make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-mesi.exe

make clean
IS_ESP=1 COH_MODE=0 CONFIG_VERBOSE=y make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-mesi-dbg.exe

# DMA
make clean
IS_ESP=1 COH_MODE=1 make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-dma.exe

make clean
IS_ESP=1 COH_MODE=1 CONFIG_VERBOSE=y make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-dma-dbg.exe

# SPX
make clean
IS_ESP=0 COH_MODE=2 make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-spx.exe

make clean
IS_ESP=0 COH_MODE=2 CONFIG_VERBOSE=y make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-spx-dbg.exe