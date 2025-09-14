# README

國立陽明交通大學資訊工程學系，「資訊工程專題」成果， RISC-V 處理器 Aquila 的 64 位元版本。關於原本的 Aquila 處理器，請參考：[https://github.com/eisl-nctu/aquila](https://github.com/eisl-nctu/aquila)

目前 repository 僅提供文件說明，程式碼尚未公開，此部分請洽 EISL@NYCU 。

## 規格

原本的 Aquila 處理器支援 rv32ima_zicsr_zifencei 的 ISA 。 64 位元版本支援 rv64ima_zicsr_zifencei 。

## 硬體

「Hardware」資料夾存放所有 HDL 程式碼。分為三個版本：

1.  tcm_version
2. dram_version
3.  mmu_version

一份「Aquila_hw.md」檔案說明與 32 位元版本相比的改動結果；另一份「Aquila64_mmu.md」說明整合 MMU 之後的改動。

## 軟體

「Software」資料夾存放所有軟體程式碼。主要包含 Aquila 原有的 C 函式庫和 bootloader ，以及一些用以測試的程式。一份 「note_sw.md」 的檔案紀錄和 32 位元處理器相比的改動內容。

## 驗證

「verilate」資料夾存放標準測試的軟體，主要來自 32 位元處理器使用的 testbench ，以及 riscv-test 。其中執行標準測試的腳本為另外撰寫的 Python 腳本，預設執行在 WSL 系統，可能需要修改以在其他系統使用。

「riscv-tests」資料夾存放 riscv-tests 的程式碼，其中 `env/v/vm.c` 有修改過配置分頁表的行為。