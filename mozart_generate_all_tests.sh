# MESI
make clean
IS_ESP=1 COH_MODE=0 make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-mesi.exe

# DMA
make clean
IS_ESP=1 COH_MODE=1 make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-dma.exe

# SPX
make clean
IS_ESP=0 COH_MODE=2 make
mv mini-era-RV-HW-F-V-Cbp.exe miniera-spandex.exe