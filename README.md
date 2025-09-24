# README

國立陽明交通大學資訊工程學系，「資訊工程專題」成果， 64 位元 RISC-V 處理器實作。

採用本校嵌入式智慧系統實驗室（EISL@NYCU）所開發的 Aquila 處理器為基礎，在既有的為架構上修改電路以支援 64 位元的指令。

關於 32 位元的 Aquila 處理器，請參考：[https://github.com/eisl-nctu/aquila](https://github.com/eisl-nctu/aquila)

## 規格

原本的 Aquila 處理器支援 rv32ima_zicsr_zifencei 的 ISA 。 64 位元版本支援 rv64ima_zicsr_zifencei 。

## 硬體

「Hardware」資料夾存放所有 HDL 程式碼。分為三個版本：

1.  tcm_version
2. dram_version
3.  mmu_version

一份「Aquila_hw.md」檔案說明與 32 位元版本相比的改動結果；另一份「Aquila64_mmu.md」說明整合 MMU 之後的改動。

## 軟體

「Software」資料夾存放所有軟體程式碼。包含 Aquila 原有的 C 函式庫和 bootloader ，以及用以測試效能的 Dhrystone benchmark 。一份「note_sw.md」的檔案紀錄和 32 位元處理器相比的改動內容。

## 驗證

以 Verilator 進行驗證。Testbench 主要來自 32 位元使用之版本， test suite 使用 [riscv-test](https://github.com/riscv-software-src/riscv-tests) 的以下指令子集：

- mi
- si
- ui
- um
- ua

測試結果請參考硬體的紀錄文件。

針對啟用虛擬記憶體的版本，修改了 `env/v/vm.c` 檔案中配置 page table 的行為。