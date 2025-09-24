# Aquila 64 Software
###### tags:`aquila`, `software`

---

# Introduction

此文章介紹 64 位元 Aquila 處理器（dram 版本）相比 32 位元處理器進行的修改。

# Elibc

大部分的 C 程式採用巨集定義變數的大小，這可以有效的避免不同暫存器寬度和 ABI 帶來的影響。而只需要依照 ABI 調整巨集。 Elibc 裡，定義這些巨集的檔案整合在 stddef.h 中，因此依照 lp64 的規範調整定義。

# Bootloader

64 位元的 uartboot 大多沿用 32 位元的版本，包含 MMIO 使用的位址在內都沒有修改。僅因應 64 位元系統，修改 elf header 和 program header 的大小和數量，也因如此目前的 uartboot 很難控制在 1 KiB 以內。

## Code Model

RISC-V 有 2 種 code model ： medlow 和 medany ，主要定義了編譯好的目標檔採用何種定址模式。 RISC-V 只支援 3 種模式：

1. 相對 PC 定址：透過 `auipc` 、 `jal` 、和分支指令實現。
2. 暫存器偏移定址：透過 `jalr` 、 `addi` 、和所有 load / store 指令實現。
3. 絕對定址：透過 `lui` 實現（這可以視為 `x0` 暫存器的偏移）。

在編譯工具鏈時，預設會採用 medlow 作為 code model ，這使程式的定址範圍只有 $\pm$ 2 GiB ，對於 32 位元是完全覆蓋，卻無法用於 64 位元。

因此編譯工具鏈須改用以下的環境：

```bash
./configure --prefix=/opt/riscv --with-arch=rv64ima_zicsr_zifencei --with-abi=lp64 --with-cmodel=medany
```

此外， 64 位元的 uartboot 採用 pseudo assembly code 提供的 `la` 取得位址。

```c
asm volatile ("la t0, sp_store");
asm volatile ("sd sp, 0(t0)");
```

`la` 指令會由 assembler 編譯為使用 `auipc` 搭配 `addi` 指令，在任意 PC 位址實現 $\pm$ 2 GiB 的跳躍。

# SPI Driver

SPI 協議 1 次只傳送 1 位元組，就是 `spi.h` 檔案裡的 `spi_txrx(unsigned char)` ，之後轉型為 4 位元組的 `unsigned int` 。 64 位元的 SPI driver 並未更動程式碼，因為調整為 8 位元組的 `unsigned long` 不會提升效能。